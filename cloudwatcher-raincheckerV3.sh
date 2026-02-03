#!/bin/bash
#==============================================================================
# AstroShell Rain Checker - Automatic Dome Protection
#==============================================================================
# Monitors Cloudwatcher Solo rain sensor and closes dome automatically.
# Sends Pushover notifications for rain alerts and connectivity issues.
#
# Target Hardware: Cloudwatcher Solo (Raspberry Pi 3)
# Dome Controller: Arduino MEGA at 192.168.1.177
#
#==============================================================================
# FUNCTIONALITY
#==============================================================================
# 1. Reads rain value from aag_json.dat every 10 seconds
# 2. If rain detected (value < threshold):
#    - Sends Pushover alert
#    - Closes both dome shutters ($3=West, $1=East)
#    - Sets RAIN_TRIGGERED flag to prevent repeated actions
# 3. If dry (value >= threshold) for 30+ minutes after rain:
#    - Sends "dry" notification (optional)
#    - Clears rain flag
# 4. Ping monitoring (rain AND dry):
#    - Alerts if dome controller unreachable
#    - Clears alarm when connectivity restored
#
#==============================================================================
# FILE LOCATIONS (read-only root filesystem)
#==============================================================================
# Script (persistent):   /usr/local/bin/cloudwatcher-rainchecker.sh
# Service (persistent):  /etc/systemd/system/cloudwatcher-rainchecker.service
# Log file (tmpfs):      /home/aagsolo/rainchecker.log
# Status flags (tmpfs):  /home/aagsolo/RAINTRIGGERED, DRYTRIGGERED, PINGALARM
# Weather data (tmpfs):  /home/aagsolo/aag_json.dat
#
#==============================================================================
# INSTALLATION
#==============================================================================
# # SSH to the Solo
# ssh root@192.168.1.151
#
# # Make root filesystem writable
# mount -o remount,rw /
#
# # Copy script
# cp cloudwatcher-raincheckerV3.sh /usr/local/bin/cloudwatcher-rainchecker.sh
# chmod +x /usr/local/bin/cloudwatcher-rainchecker.sh
#
# # Create systemd service (see cloudwatcher-rainchecker.service)
# cp cloudwatcher-rainchecker.service /etc/systemd/system/
# systemctl daemon-reload
# systemctl enable cloudwatcher-rainchecker
#
# # Protect filesystem
# mount -o remount,ro /
#
# # Start service
# systemctl start cloudwatcher-rainchecker
#
#==============================================================================
# SERVICE MANAGEMENT
#==============================================================================
# Start:    systemctl start cloudwatcher-rainchecker
# Stop:     systemctl stop cloudwatcher-rainchecker
# Restart:  systemctl restart cloudwatcher-rainchecker
# Status:   systemctl status cloudwatcher-rainchecker
# Logs:     journalctl -u cloudwatcher-rainchecker -f
#           tail -f /home/aagsolo/rainchecker.log
#
#==============================================================================
# CONFIGURATION
#==============================================================================

# --- File Paths ---
STATUS_FILE="/home/aagsolo/aag_json.dat"
RAIN_TRIGGERED="/home/aagsolo/RAINTRIGGERED"
DRY_TRIGGERED="/home/aagsolo/DRYTRIGGERED"
PING_ALARM="/home/aagsolo/PINGALARM"
LAST_RAIN_TIME="/home/aagsolo/LASTRAINTIME"
LOG_FILE="/home/aagsolo/rainchecker.log"

# --- Thresholds ---
# Rain detected when value falls BELOW this threshold
RAIN_THRESHOLD=2900

# Minutes of continuous dry weather before sending "dry" notification
DRY_COOLDOWN_MINUTES=30

# --- Log Configuration ---
MAX_LOG_LINES=5000

# --- Pushover Configuration ---
PUSHOVER_TOKEN="xxxxx"
PUSHOVER_USER="yyyyy"
PUSHOVER_API_URL="https://api.pushover.net/1/messages.json"
# Optional: Dashboard URL included in notifications (comment out to disable)
# PUSHOVER_INFO_URL="http://your-dashboard:8888"

# --- Dome Controller ---
DOME_IP="192.168.1.177"

# --- Timing ---
CHECK_INTERVAL=10       # Seconds between checks
RAIN_ACTION_COOLDOWN=300  # Seconds to wait after dome close action (5 min)

#==============================================================================
# LOGGING FUNCTIONS
#==============================================================================

log_message() {
    local message="$1"
    local timestamp
    timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    echo "$timestamp: $message"
    echo "$timestamp: $message" >> "$LOG_FILE"
}

trim_log_file() {
    if [[ -f "$LOG_FILE" ]]; then
        local line_count
        line_count=$(wc -l < "$LOG_FILE" | tr -d ' ')
        if (( line_count > MAX_LOG_LINES )); then
            log_message "Trimming log file to $MAX_LOG_LINES lines (was: $line_count)"
            local temp_file
            temp_file=$(mktemp "${LOG_FILE}.XXXXXX")
            if tail -n "$MAX_LOG_LINES" "$LOG_FILE" > "$temp_file"; then
                mv "$temp_file" "$LOG_FILE"
            else
                log_message "ERROR: Failed to trim log file"
                rm -f "$temp_file"
            fi
        fi
    fi
}

#==============================================================================
# HELPER FUNCTIONS
#==============================================================================

# Extract rain value from JSON file
get_rain_value() {
    if [[ ! -f "$STATUS_FILE" ]] || [[ ! -r "$STATUS_FILE" ]]; then
        log_message "ERROR: Status file $STATUS_FILE not found or not readable"
        echo ""
        return 1
    fi

    local value
    if command -v jq > /dev/null; then
        value=$(jq -r '.rain // empty' "$STATUS_FILE" 2>/dev/null)
    else
        value=$(grep -o '"rain"[[:space:]]*:[[:space:]]*[0-9]*' "$STATUS_FILE" | grep -o '[0-9]*$')
    fi

    if [[ "$value" =~ ^[0-9]+$ ]]; then
        echo "$value"
        return 0
    else
        log_message "ERROR: Invalid rain value: '$value'"
        echo ""
        return 1
    fi
}

# Check dome status via HTTP
check_dome_status() {
    local status
    status=$(curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/?\$S" 2>/dev/null | tr -d '\r\n\t ' | tr '[:lower:]' '[:upper:]')

    if [[ -z "$status" ]]; then
        echo "ERROR"
        return 1
    fi

    echo "$status"
    return 0
}

# Ping test for dome controller
check_dome_ping() {
    if ping -c 1 -W 3 "$DOME_IP" > /dev/null 2>&1; then
        return 0  # Success
    else
        return 1  # Failed
    fi
}

# Send Pushover notification
send_pushover() {
    local title="$1"
    local message="$2"
    local priority="${3:-0}"

    local cmd="curl -s --form-string \"token=$PUSHOVER_TOKEN\" \
                      --form-string \"user=$PUSHOVER_USER\" \
                      --form-string \"title=$title\" \
                      --form-string \"message=$message\" \
                      --form-string \"priority=$priority\""

    # Add URL if configured
    if [[ -n "${PUSHOVER_INFO_URL:-}" ]]; then
        cmd="$cmd --form-string \"url=$PUSHOVER_INFO_URL\""
    fi

    eval "$cmd \"$PUSHOVER_API_URL\"" > /dev/null 2>&1
}

# Get current Unix timestamp
get_timestamp() {
    date +%s
}

# Check if enough time has passed since last rain
is_dry_cooldown_passed() {
    if [[ ! -f "$LAST_RAIN_TIME" ]]; then
        return 0  # No rain recorded, cooldown passed
    fi

    local last_rain
    last_rain=$(cat "$LAST_RAIN_TIME" 2>/dev/null)

    if [[ ! "$last_rain" =~ ^[0-9]+$ ]]; then
        return 0  # Invalid timestamp, assume cooldown passed
    fi

    local now
    now=$(get_timestamp)
    local elapsed=$(( (now - last_rain) / 60 ))  # Minutes

    if (( elapsed >= DRY_COOLDOWN_MINUTES )); then
        return 0  # Cooldown passed
    else
        return 1  # Still in cooldown
    fi
}

#==============================================================================
# CLEANUP ON EXIT
#==============================================================================

cleanup() {
    log_message "Rain checker script stopping (received signal)"
    exit 0
}

trap cleanup SIGTERM SIGINT SIGHUP

#==============================================================================
# MAIN SCRIPT
#==============================================================================

log_message "=============================================="
log_message "Rain checker script started"
log_message "Rain threshold: < $RAIN_THRESHOLD"
log_message "Dry cooldown: $DRY_COOLDOWN_MINUTES minutes"
log_message "Dome IP: $DOME_IP"
log_message "=============================================="

while true; do
    # --- Get Rain Value ---
    RAIN_VALUE=$(get_rain_value)

    if [[ -z "$RAIN_VALUE" ]]; then
        log_message "Warning: Could not read rain value, skipping cycle"
        sleep "$CHECK_INTERVAL"
        trim_log_file
        continue
    fi

    # --- Ping Check (always, rain or dry) ---
    if ! check_dome_ping; then
        if [[ ! -f "$PING_ALARM" ]]; then
            log_message "PING FAILED to $DOME_IP - sending alarm!"
            send_pushover "Dome Ping Error" "Cannot reach dome at $DOME_IP! Rain value: $RAIN_VALUE" 1
            touch "$PING_ALARM"
        fi
    else
        if [[ -f "$PING_ALARM" ]]; then
            log_message "Ping to $DOME_IP restored"
            rm -f "$PING_ALARM"
        fi
    fi

    # --- Rain Detection ---
    if (( RAIN_VALUE < RAIN_THRESHOLD )); then
        # RAIN DETECTED
        log_message "Rain detected! Value: $RAIN_VALUE (threshold: $RAIN_THRESHOLD)"

        # Record rain time for cooldown calculation
        get_timestamp > "$LAST_RAIN_TIME"

        # Clear dry flag (we're in rain now)
        rm -f "$DRY_TRIGGERED"

        if [[ ! -f "$RAIN_TRIGGERED" ]]; then
            # First rain detection - take action!
            log_message "=== RAIN ALERT - CLOSING DOME ==="

            # Check current dome status
            DOME_STATUS=$(check_dome_status)
            log_message "Dome status: $DOME_STATUS"

            if [[ "$DOME_STATUS" == "CLOSED" ]]; then
                log_message "Dome already closed, sending notification only"
                send_pushover "Rain Alert" "Rain detected (Value: $RAIN_VALUE). Dome already closed." 1
            else
                # Send alert
                send_pushover "RAIN! Closing Dome" "Rain value: $RAIN_VALUE - Closing dome now!" 1

                # Close dome (West first, then East)
                log_message "Closing WEST shutter ($DOME_IP/?\$3)"
                curl -s --max-time 10 "http://$DOME_IP/?\$3" > /dev/null 2>&1
                sleep 3

                log_message "Closing EAST shutter ($DOME_IP/?\$1)"
                curl -s --max-time 10 "http://$DOME_IP/?\$1" > /dev/null 2>&1

                log_message "Dome close commands sent"
            fi

            # Set rain flag
            touch "$RAIN_TRIGGERED"

            # Wait before next action cycle
            log_message "Waiting $((RAIN_ACTION_COOLDOWN / 60)) minutes before next rain action"
            sleep "$RAIN_ACTION_COOLDOWN"
            continue
        else
            # Rain continues, flag already set
            log_message "Rain continues (Value: $RAIN_VALUE), dome already triggered"
        fi

    else
        # DRY
        if [[ -f "$RAIN_TRIGGERED" ]]; then
            # Was raining, now dry - check cooldown
            if is_dry_cooldown_passed; then
                log_message "Dry for $DRY_COOLDOWN_MINUTES+ minutes (Value: $RAIN_VALUE)"

                if [[ ! -f "$DRY_TRIGGERED" ]]; then
                    # Send dry notification (only once)
                    send_pushover "Weather Clear" "Dry for $DRY_COOLDOWN_MINUTES+ minutes. Rain value: $RAIN_VALUE" 0
                    touch "$DRY_TRIGGERED"
                fi

                # Clear rain flag
                rm -f "$RAIN_TRIGGERED"
                rm -f "$LAST_RAIN_TIME"
            else
                # Still in cooldown period
                local remaining
                if [[ -f "$LAST_RAIN_TIME" ]]; then
                    local last_rain=$(cat "$LAST_RAIN_TIME")
                    local now=$(get_timestamp)
                    remaining=$(( DRY_COOLDOWN_MINUTES - ((now - last_rain) / 60) ))
                    log_message "Dry (Value: $RAIN_VALUE) but in cooldown, ${remaining}min remaining"
                fi
            fi
        fi
        # If never rained (no RAIN_TRIGGERED), just continue silently
    fi

    sleep "$CHECK_INTERVAL"
    trim_log_file
done
