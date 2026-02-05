#!/bin/bash
#==============================================================================
# AstroShell Motor Tick Statistic Batch Runner
#==============================================================================
# Automated dome open/close cycles to collect motor tick runtime data.
# Designed to run manually on the Cloudwatcher Solo (Pi3) during daytime.
#
# Target Hardware: Cloudwatcher Solo (Raspberry Pi 3)
# Dome Controller: Arduino MEGA at 192.168.1.177
#
#==============================================================================
# PURPOSE
#==============================================================================
# Motor runtime (in ISR ticks) varies with temperature due to motor oil
# viscosity changes. This script collects statistical data by repeatedly
# opening and closing the dome throughout the day.
#
# One "run" = dome starts CLOSED -> OPEN -> CLOSED (one full open/close cycle).
# Default: 20 runs per batch session (configurable via MAX_RUNS).
#
# Rest periods between direction changes are RANDOMIZED (5-30 minutes) to
# provide statistical evidence of temperature-dependent motor behavior:
# - Short rest (5 min): motor oil still warm from previous run
# - Long rest (30 min): motor oil cools down to ambient temperature
# - Random distribution provides data points across the cooling curve
#
# The Arduino's tick logger (astroshell_ticklogger.py on port 88) records
# each motor run with timestamp and ambient temperature for correlation
# analysis.
#
#==============================================================================
# BEHAVIOR
#==============================================================================
# 1. Enable tick logging on Arduino (if not already enabled)
# 2. Verify dome is at CLOSED endpoint (both shutters fully closed)
# 3. Check weather (rain sensor) - only operate when dry
# 4. OPEN both shutters (West first, then East)
# 5. Wait 200 seconds for motors to complete and reach endpoints
# 6. Verify both shutters at OPEN endpoint (Pushover alert if not)
# 7. Rest for random 5-30 minutes (rain checked during rest)
# 8. Check dome state - may have been closed externally during rest
# 9. CLOSE both shutters (no weather check - closing is always safe)
# 10. Wait 200 seconds for motors to complete and reach endpoints
# 11. Verify both shutters at CLOSED endpoint (Pushover alert if not)
# 12. Rest for random 5-30 minutes
# 13. Repeat from step 3 until MAX_RUNS completed
# 14. Send final summary and exit
#
# Weather check strategy: Rain is only checked before OPENING the dome.
# Closing is always safe regardless of weather. During the open rest period,
# rain is monitored and the dome closed immediately if detected. If the
# rain checker or manual intervention closes the dome during rest, the
# script detects this and skips the close (run does not count).
#
# Endpoint verification: After each motor run, the script parses the Arduino
# web page to confirm BOTH shutters reached their target limit switch.
# "Intermediate" state on any shutter triggers an INTERRUPT alert.
#
# Rain during operation: Script pauses, closes dome if open, waits for dry.
# Interrupted motor run: Pushover alert with details. Run does not count.
# Ctrl+C: Closes dome safely and sends shutdown notification.
#
#==============================================================================
# PUSHOVER NOTIFICATIONS
#==============================================================================
# Every notification includes "Run X/Y" (current run / total planned).
#
# - Script started / completed / stopped (Ctrl+C)
# - Dome opening (with run number, temperature)
# - Dome open confirmed at endpoints (or INTERRUPTED alert)
# - Dome closing (with run number, rest time used, temperature)
# - Dome closed confirmed at endpoints (or INTERRUPTED alert)
# - Rain detected (pausing) / cleared (resuming)
# - Rest period info (random minutes chosen)
#
#==============================================================================
# PREREQUISITES
#==============================================================================
# - astroshell_ticklogger.py running on Solo port 88
# - Rain checker (cloudwatcher-raincheckerV3.sh) may remain running for
#   safety. This script cooperates by checking rain itself and will detect
#   if the rain checker intervenes.
# - aag_json.dat must be updated by Cloudwatcher software
#
#==============================================================================
# USAGE
#==============================================================================
# ssh root@192.168.1.151
# /usr/local/bin/motortick-statistic-batchrun.sh
#
# Stop with Ctrl+C (dome will be closed safely)
#
#==============================================================================
# INSTALLATION
#==============================================================================
# mount -o remount,rw /
# cp motortick-statistic-batchrun.sh /usr/local/bin/
# chmod +x /usr/local/bin/motortick-statistic-batchrun.sh
# mount -o remount,ro /
#
#==============================================================================
# CONFIGURATION
#==============================================================================

# --- File Paths ---
STATUS_FILE="/home/aagsolo/aag_json.dat"
LOG_FILE="/home/aagsolo/batchrun.log"

# --- Dome Controller ---
DOME_IP="192.168.1.177"

# --- Run Count ---
# One run = dome CLOSED -> OPEN -> CLOSED (one full cycle)
MAX_RUNS=20

# --- Rain Threshold ---
# Rain detected when value falls BELOW this threshold (same as rain checker)
RAIN_THRESHOLD=2900

# --- Motor Timing ---
# Time to wait for motors to reach endpoints after command (seconds).
# Motor timeout = 6527 ticks / 61 Hz = 107 seconds.
# 200 seconds provides generous margin for both motors to complete
# and for the Arduino to push tick data to the logger.
MOTOR_WAIT_SECONDS=200

# --- Rest Period Range (minutes) ---
# Randomized to collect data at varying motor oil temperatures.
# Minimum 5 minutes ensures motor has stopped and data is pushed.
MIN_REST_MINUTES=5
MAX_REST_MINUTES=30

# --- Rain Wait ---
# Seconds between rain checks when waiting for dry weather
RAIN_CHECK_INTERVAL=60

# --- Pushover Configuration ---
PUSHOVER_TOKEN="xxxxx"
PUSHOVER_USER="yyyyy"
PUSHOVER_API_URL="https://api.pushover.net/1/messages.json"

# --- Log Configuration ---
MAX_LOG_LINES=5000

#==============================================================================
# COUNTERS
#==============================================================================
COMPLETED_RUNS=0
OPEN_COUNT=0
CLOSE_COUNT=0
INTERRUPT_COUNT=0
START_TIME=""

#==============================================================================
# LOGGING
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
            local temp_file
            temp_file=$(mktemp "${LOG_FILE}.XXXXXX")
            if tail -n "$MAX_LOG_LINES" "$LOG_FILE" > "$temp_file"; then
                mv "$temp_file" "$LOG_FILE"
            else
                rm -f "$temp_file"
            fi
        fi
    fi
}

#==============================================================================
# HELPER FUNCTIONS
#==============================================================================

# Current run label for messages, e.g. "Run 3/20"
run_label() {
    local current_run=$((COMPLETED_RUNS + 1))
    echo "Run ${current_run}/${MAX_RUNS}"
}

# Send Pushover notification
send_pushover() {
    local title="$1"
    local message="$2"
    local priority="${3:-0}"

    log_message "Pushover: [$title] $message"

    curl -s --connect-timeout 10 --max-time 15 \
        --form-string "token=$PUSHOVER_TOKEN" \
        --form-string "user=$PUSHOVER_USER" \
        --form-string "title=$title" \
        --form-string "message=$message" \
        --form-string "priority=$priority" \
        "$PUSHOVER_API_URL" > /dev/null 2>&1
}

# Get rain value from aag_json.dat
get_rain_value() {
    if [[ ! -f "$STATUS_FILE" ]] || [[ ! -r "$STATUS_FILE" ]]; then
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
        echo ""
        return 1
    fi
}

# Check if it's currently dry
is_dry() {
    local rain_value
    rain_value=$(get_rain_value)

    if [[ -z "$rain_value" ]]; then
        log_message "WARNING: Cannot read rain value"
        return 1  # Assume not dry if we can't read
    fi

    if (( rain_value >= RAIN_THRESHOLD )); then
        return 0  # Dry
    else
        return 1  # Raining
    fi
}

# Get dome status via HTTP ($S command)
# Returns: "OPEN", "CLOSED", or "ERROR"
get_dome_status() {
    local status
    status=$(curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/?\$S" 2>/dev/null | tr -d '\r\n\t ' | tr '[:lower:]' '[:upper:]')

    if [[ -z "$status" ]]; then
        echo "ERROR"
        return 1
    fi

    echo "$status"
    return 0
}

# Get detailed shutter endpoint status by parsing the Arduino web page.
# Counts "Physically OPEN" and "Physically CLOSED" State lines (2 shutters).
# Returns: "BOTH_OPEN", "BOTH_CLOSED", or detail string like
#          "1xOPEN 0xCLOSED 1xINTERMEDIATE"
get_shutter_details() {
    local page
    page=$(curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/" 2>/dev/null)

    if [[ -z "$page" ]]; then
        echo "ERROR"
        return 1
    fi

    # Count State lines for each position.
    # "Physically OPEN" / "Physically CLOSED" only appear in the State display,
    # not in the limit switch table (which uses "Phys." abbreviation).
    local open_count closed_count intermediate_count
    open_count=$(echo "$page" | grep -c "Physically OPEN")
    closed_count=$(echo "$page" | grep -c "Physically CLOSED")
    intermediate_count=$(echo "$page" | grep -c "Intermediate")

    if (( open_count == 2 && intermediate_count == 0 )); then
        echo "BOTH_OPEN"
    elif (( closed_count == 2 && intermediate_count == 0 )); then
        echo "BOTH_CLOSED"
    else
        echo "${open_count}xOPEN ${closed_count}xCLOSED ${intermediate_count}xINTERMEDIATE"
    fi
    return 0
}

# Check if tick logging is enabled on the Arduino
is_tick_logging_enabled() {
    local page
    page=$(curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/" 2>/dev/null)

    if echo "$page" | grep -q '<strong>ENABLED</strong>'; then
        return 0
    else
        return 1
    fi
}

# Enable tick logging on Arduino (toggle via $L)
ensure_tick_logging() {
    if is_tick_logging_enabled; then
        log_message "Tick logging already enabled"
        return 0
    fi

    log_message "Enabling tick logging on Arduino..."
    curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/?\$L" > /dev/null 2>&1
    sleep 2

    if is_tick_logging_enabled; then
        log_message "Tick logging enabled successfully"
        return 0
    else
        log_message "WARNING: Could not verify tick logging state"
        return 1
    fi
}

# Get ambient temperature from aag_json.dat
get_temperature() {
    if [[ ! -f "$STATUS_FILE" ]]; then
        echo "?"
        return
    fi

    local temp
    if command -v jq > /dev/null; then
        temp=$(jq -r '.temp // "?"' "$STATUS_FILE" 2>/dev/null)
    else
        temp=$(grep -o '"temp"[[:space:]]*:[[:space:]]*[0-9.-]*' "$STATUS_FILE" | grep -o '[0-9.-]*$')
    fi

    echo "${temp:-?}"
}

# Generate random rest time in minutes (MIN_REST_MINUTES to MAX_REST_MINUTES)
random_rest_minutes() {
    local range=$((MAX_REST_MINUTES - MIN_REST_MINUTES + 1))
    echo $((RANDOM % range + MIN_REST_MINUTES))
}

# Open dome: West ($4) first, then East ($2)
open_dome() {
    log_message ">>> OPENING dome: West (\$4) then East (\$2)"

    curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/?\$4" > /dev/null 2>&1
    sleep 3
    curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/?\$2" > /dev/null 2>&1

    log_message "Open commands sent, waiting ${MOTOR_WAIT_SECONDS}s for motors to reach endpoints..."
    sleep "$MOTOR_WAIT_SECONDS"
}

# Close dome: West ($3) first, then East ($1)
close_dome() {
    log_message ">>> CLOSING dome: West (\$3) then East (\$1)"

    curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/?\$3" > /dev/null 2>&1
    sleep 3
    curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/?\$1" > /dev/null 2>&1

    log_message "Close commands sent, waiting ${MOTOR_WAIT_SECONDS}s for motors to reach endpoints..."
    sleep "$MOTOR_WAIT_SECONDS"
}

# Wait for dry weather, checking periodically
wait_for_dry() {
    local rain_notified=false

    while ! is_dry; do
        if [[ "$rain_notified" == "false" ]]; then
            local rain_value
            rain_value=$(get_rain_value)
            log_message "Rain detected (value: ${rain_value}), pausing batch run"
            send_pushover "Batch: Rain Pause" "$(run_label): Rain detected (value: ${rain_value}). Paused, waiting for dry." 0
            rain_notified=true
        fi
        sleep "$RAIN_CHECK_INTERVAL"
    done

    if [[ "$rain_notified" == "true" ]]; then
        local rain_value
        rain_value=$(get_rain_value)
        log_message "Weather cleared (value: ${rain_value}), resuming"
        send_pushover "Batch: Dry - Resuming" "$(run_label): Weather cleared (value: ${rain_value}). Resuming." 0
    fi
}

# Interruptible sleep: checks rain every 60 seconds during rest period
# Returns 0 if completed, 1 if rain interrupted
rest_with_rain_check() {
    local total_seconds=$1
    local elapsed=0

    while (( elapsed < total_seconds )); do
        local chunk=$RAIN_CHECK_INTERVAL
        if (( elapsed + chunk > total_seconds )); then
            chunk=$((total_seconds - elapsed))
        fi
        sleep "$chunk"
        elapsed=$((elapsed + chunk))

        if ! is_dry; then
            return 1  # Rain interrupted rest
        fi
    done

    return 0  # Completed without rain
}

# Format elapsed time as "Xh Ym"
format_elapsed() {
    local seconds=$1
    local hours=$((seconds / 3600))
    local minutes=$(( (seconds % 3600) / 60 ))
    if (( hours > 0 )); then
        echo "${hours}h ${minutes}m"
    else
        echo "${minutes}m"
    fi
}

#==============================================================================
# CLEANUP ON EXIT
#==============================================================================

cleanup() {
    log_message "=============================================="
    log_message "Batch run stopping (received signal)"

    local now
    now=$(date +%s)
    local elapsed=$((now - START_TIME))

    # Close dome safely
    log_message "Safety: closing dome before exit"
    close_dome

    local final_details
    final_details=$(get_shutter_details)
    log_message "Final shutter state: $final_details"

    local summary="Completed: ${COMPLETED_RUNS}/${MAX_RUNS} runs, Opens: $OPEN_COUNT, Closes: $CLOSE_COUNT, Interrupts: $INTERRUPT_COUNT, Runtime: $(format_elapsed $elapsed)"
    log_message "Summary: $summary"

    send_pushover "Batch: Stopped" "Batch run stopped (Ctrl+C). $summary. Shutters: $final_details" 0
    log_message "=============================================="
    exit 0
}

trap cleanup SIGTERM SIGINT SIGHUP

#==============================================================================
# MAIN SCRIPT
#==============================================================================

START_TIME=$(date +%s)

log_message "=============================================="
log_message "Motor Tick Statistic Batch Runner starting"
log_message "Dome IP: $DOME_IP"
log_message "Planned runs: $MAX_RUNS"
log_message "Rain threshold: < $RAIN_THRESHOLD"
log_message "Motor wait: ${MOTOR_WAIT_SECONDS}s"
log_message "Rest range: ${MIN_REST_MINUTES}-${MAX_REST_MINUTES} minutes (randomized)"
log_message "=============================================="

# --- Pre-flight checks ---

# Check dome controller reachable
if ! ping -c 1 -W 3 "$DOME_IP" > /dev/null 2>&1; then
    log_message "FATAL: Cannot reach dome at $DOME_IP"
    send_pushover "Batch: FAILED" "Cannot reach dome at $DOME_IP. Batch run aborted." 1
    exit 1
fi

# Check weather data available
if [[ ! -f "$STATUS_FILE" ]]; then
    log_message "FATAL: Weather data file $STATUS_FILE not found"
    send_pushover "Batch: FAILED" "Weather data not available. Batch run aborted." 1
    exit 1
fi

# Enable tick logging
ensure_tick_logging

# Get initial state
DOME_STATUS=$(get_dome_status)
SHUTTER_DETAILS=$(get_shutter_details)
TEMPERATURE=$(get_temperature)
RAIN_VALUE=$(get_rain_value)
log_message "Initial state: Dome=$DOME_STATUS, Shutters=$SHUTTER_DETAILS, Temp=${TEMPERATURE}C, Rain=$RAIN_VALUE"

send_pushover "Batch: Started" "Motor tick batch: ${MAX_RUNS} runs planned. Dome: $DOME_STATUS ($SHUTTER_DETAILS), Temp: ${TEMPERATURE}C, Rain: $RAIN_VALUE. Rest: ${MIN_REST_MINUTES}-${MAX_REST_MINUTES}min random." 0

# --- Ensure dome starts at CLOSED endpoints ---
if [[ "$SHUTTER_DETAILS" != "BOTH_CLOSED" ]]; then
    log_message "Dome not at closed endpoints ($SHUTTER_DETAILS), closing first..."
    send_pushover "Batch: Initial Close" "Dome is $SHUTTER_DETAILS, closing to endpoints before starting." 0
    close_dome

    SHUTTER_DETAILS=$(get_shutter_details)
    if [[ "$SHUTTER_DETAILS" != "BOTH_CLOSED" ]]; then
        log_message "FATAL: Dome still $SHUTTER_DETAILS after initial close"
        send_pushover "Batch: FAILED" "Cannot reach closed endpoints ($SHUTTER_DETAILS). Batch aborted. Check manually!" 1
        exit 1
    fi
    log_message "Dome at closed endpoints, ready to start"
fi

# --- Main Loop ---
while (( COMPLETED_RUNS < MAX_RUNS )); do
    trim_log_file

    TEMPERATURE=$(get_temperature)
    log_message "--- $(run_label) start (Temp: ${TEMPERATURE}C) ---"

    # ===== STEP 1: Check rain =====
    wait_for_dry

    # ===== STEP 2: Verify dome is at CLOSED endpoints =====
    SHUTTER_DETAILS=$(get_shutter_details)
    if [[ "$SHUTTER_DETAILS" != "BOTH_CLOSED" ]]; then
        log_message "WARNING: Not at closed endpoints ($SHUTTER_DETAILS), closing first"
        close_dome
        SHUTTER_DETAILS=$(get_shutter_details)
        if [[ "$SHUTTER_DETAILS" != "BOTH_CLOSED" ]]; then
            log_message "ERROR: Cannot reach closed endpoints ($SHUTTER_DETAILS), skipping run"
            INTERRUPT_COUNT=$((INTERRUPT_COUNT + 1))
            send_pushover "Batch: INTERRUPT" "$(run_label): Cannot reach closed endpoints ($SHUTTER_DETAILS). Skipping, will retry." 1
            sleep "$RAIN_CHECK_INTERVAL"
            continue
        fi
    fi

    # ===== STEP 3: OPEN dome =====
    TEMPERATURE=$(get_temperature)
    send_pushover "Batch: Opening" "$(run_label): Opening dome. Temp: ${TEMPERATURE}C" 0
    open_dome
    OPEN_COUNT=$((OPEN_COUNT + 1))

    # ===== STEP 4: Verify both shutters at OPEN endpoints =====
    SHUTTER_DETAILS=$(get_shutter_details)
    if [[ "$SHUTTER_DETAILS" != "BOTH_OPEN" ]]; then
        INTERRUPT_COUNT=$((INTERRUPT_COUNT + 1))
        log_message "INTERRUPTED: After open, shutters are $SHUTTER_DETAILS (expected BOTH_OPEN)"
        send_pushover "Batch: INTERRUPTED" "$(run_label): Open failed! Shutters: $SHUTTER_DETAILS (expected BOTH_OPEN). Temp: ${TEMPERATURE}C. Closing dome, run does not count." 1
        # Close dome and retry this run
        close_dome
        sleep 30
        continue
    fi
    log_message "Dome at open endpoints ($SHUTTER_DETAILS)"
    send_pushover "Batch: Opened" "$(run_label): Both shutters at OPEN endpoints. Temp: ${TEMPERATURE}C" 0

    # ===== STEP 5: Rest (random delay, dome open) =====
    REST_MINUTES=$(random_rest_minutes)
    REST_SECONDS=$((REST_MINUTES * 60))
    log_message "Resting $REST_MINUTES minutes (open) before closing..."
    send_pushover "Batch: Resting" "$(run_label): Open rest ${REST_MINUTES}min. Temp: ${TEMPERATURE}C" -1

    if ! rest_with_rain_check "$REST_SECONDS"; then
        # Rain during rest while dome is open - close immediately
        log_message "Rain during open rest! Closing dome."
        send_pushover "Batch: Rain!" "$(run_label): Rain while dome open! Closing dome." 1
        close_dome
        CLOSE_COUNT=$((CLOSE_COUNT + 1))

        SHUTTER_DETAILS=$(get_shutter_details)
        if [[ "$SHUTTER_DETAILS" != "BOTH_CLOSED" ]]; then
            INTERRUPT_COUNT=$((INTERRUPT_COUNT + 1))
            send_pushover "Batch: INTERRUPTED" "$(run_label): Rain close incomplete ($SHUTTER_DETAILS)! Check manually." 1
        else
            send_pushover "Batch: Rain Closed" "$(run_label): Dome closed after rain ($SHUTTER_DETAILS). Waiting for dry." 0
        fi

        wait_for_dry
        # Run does not count - dome was closed by rain, not a clean cycle
        continue
    fi

    # ===== STEP 6: Check dome state after rest =====
    # During the open rest, the dome may have been closed externally
    # (rain checker, manual intervention). Check before trying to close.
    SHUTTER_DETAILS=$(get_shutter_details)
    if [[ "$SHUTTER_DETAILS" == "BOTH_CLOSED" ]]; then
        # Dome was closed externally during rest - not a hardware failure
        log_message "Dome already closed during rest - external intervention"
        send_pushover "Batch: External Close" "$(run_label): Dome was closed during open rest (rain checker? manual?). Run does not count." 0
        continue
    elif [[ "$SHUTTER_DETAILS" != "BOTH_OPEN" ]]; then
        # Mixed/intermediate state - something went wrong
        INTERRUPT_COUNT=$((INTERRUPT_COUNT + 1))
        log_message "UNEXPECTED: Shutters in mixed state after rest ($SHUTTER_DETAILS)"
        send_pushover "Batch: UNEXPECTED" "$(run_label): Shutters $SHUTTER_DETAILS after open rest! Closing dome, run does not count." 1
        close_dome
        continue
    fi

    # ===== STEP 7: CLOSE dome =====
    # No weather check needed - closing is always safe regardless of weather.
    TEMPERATURE=$(get_temperature)
    send_pushover "Batch: Closing" "$(run_label): Closing dome. Open rest was ${REST_MINUTES}min. Temp: ${TEMPERATURE}C" 0
    close_dome
    CLOSE_COUNT=$((CLOSE_COUNT + 1))

    # ===== STEP 8: Verify both shutters at CLOSED endpoints =====
    SHUTTER_DETAILS=$(get_shutter_details)
    if [[ "$SHUTTER_DETAILS" != "BOTH_CLOSED" ]]; then
        INTERRUPT_COUNT=$((INTERRUPT_COUNT + 1))
        log_message "INTERRUPTED: After close, shutters are $SHUTTER_DETAILS (expected BOTH_CLOSED)"
        send_pushover "Batch: INTERRUPTED" "$(run_label): Close failed! Shutters: $SHUTTER_DETAILS (expected BOTH_CLOSED). Temp: ${TEMPERATURE}C. Retrying close." 1
        # Retry close
        close_dome
        SHUTTER_DETAILS=$(get_shutter_details)
        if [[ "$SHUTTER_DETAILS" != "BOTH_CLOSED" ]]; then
            send_pushover "Batch: CLOSE FAILED" "$(run_label): Still $SHUTTER_DETAILS after retry! Check manually. Run does not count." 1
            sleep 30
            continue
        fi
        log_message "Dome closed on retry"
    fi

    # ===== RUN COMPLETE =====
    COMPLETED_RUNS=$((COMPLETED_RUNS + 1))
    TEMPERATURE=$(get_temperature)
    log_message "Dome at closed endpoints ($SHUTTER_DETAILS) - run $COMPLETED_RUNS/$MAX_RUNS complete"

    # Check if all runs done
    if (( COMPLETED_RUNS >= MAX_RUNS )); then
        send_pushover "Batch: Run Done" "Run ${COMPLETED_RUNS}/${MAX_RUNS} complete (LAST RUN). BOTH_CLOSED. Temp: ${TEMPERATURE}C. Batch finishing." 0
        break
    fi

    # Compute next rest time and include in completion message
    REST_MINUTES=$(random_rest_minutes)
    REST_SECONDS=$((REST_MINUTES * 60))
    send_pushover "Batch: Run Done" "Run ${COMPLETED_RUNS}/${MAX_RUNS} complete. BOTH_CLOSED. Temp: ${TEMPERATURE}C. Next closed rest: ${REST_MINUTES}min." 0

    # ===== STEP 9: Rest (random delay, dome closed) =====
    log_message "Resting $REST_MINUTES minutes (closed) before next run..."

    # During closed rest, rain doesn't matter (dome is closed)
    sleep "$REST_SECONDS"
done

#==============================================================================
# BATCH COMPLETE
#==============================================================================

NOW=$(date +%s)
ELAPSED=$((NOW - START_TIME))
TEMPERATURE=$(get_temperature)

log_message "=============================================="
log_message "ALL $MAX_RUNS RUNS COMPLETED"
log_message "Opens: $OPEN_COUNT, Closes: $CLOSE_COUNT, Interrupts: $INTERRUPT_COUNT"
log_message "Total runtime: $(format_elapsed $ELAPSED)"
log_message "=============================================="

send_pushover "Batch: COMPLETE" "All ${MAX_RUNS} runs finished! Opens: $OPEN_COUNT, Closes: $CLOSE_COUNT, Interrupts: $INTERRUPT_COUNT. Runtime: $(format_elapsed $ELAPSED). Temp: ${TEMPERATURE}C. Dome closed." 0
