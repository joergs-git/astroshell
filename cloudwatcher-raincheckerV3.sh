#!/bin/bash
# Script is to be started permanently in background using the following command
#     nohup /home/aagsolo/rainchecker.sh > /dev/null 2>&1 &
# 
# --- Configuration ---
#STATUS_FILE="/home/aagsolo/aag_json_test.dat"
STATUS_FILE="/home/aagsolo/aag_json.dat"
RAIN_TRIGGERED="/home/aagsolo/RAINTRIGGERED"
DRY_TRIGGERED="/home/aagsolo/DRYTRIGGERED"
PING_ALARM="/home/aagsolo/PINGALARM"

# Rain threshold: Actions are triggered when rain value falls *below* this value.
RAIN_THRESHOLD=2900

# Log file configuration
LOG_FILE="/home/aagsolo/rainchecker.log"
MAX_LOG_LINES=5000

# Pushover configuration (better to define as variables above for security reasons)
PUSHOVER_TOKEN="xxxxx"
PUSHOVER_USER="yyyyy"
PUSHOVER_API_URL="https://api.pushover.net/1/messages.json"
PUSHOVER_INFO_URL="http://94.zz.xx.yy:8888"

# Target IP for dome control
DOME_IP="192.168.1.177"

# --- Logging Functions ---
log_message() {
    local message="$1"
    local timestamped_message
    timestamped_message="$(date '+%Y-%m-%d %H:%M:%S'): $message"
    echo "$timestamped_message" # Output to console
    echo "$timestamped_message" >> "$LOG_FILE" # Output to log file
}

trim_log_file() {
    if [[ -f "$LOG_FILE" ]]; then
        local line_count
        line_count=$(wc -l < "$LOG_FILE" | tr -d ' ') # Ensure only the number is read
        if (( line_count > MAX_LOG_LINES )); then
            log_message "Log file $LOG_FILE will be trimmed to the last $MAX_LOG_LINES lines (Current: $line_count lines)."
            local temp_log_file
            temp_log_file=$(mktemp "${LOG_FILE}.XXXXXX") # Safe temporary filename
            if tail -n "$MAX_LOG_LINES" "$LOG_FILE" > "$temp_log_file"; then
                mv "$temp_log_file" "$LOG_FILE"
            else
                log_message "ERROR: Trimming log file $LOG_FILE failed."
                rm -f "$temp_log_file" # Delete temporary file in case of error
            fi
        fi
    fi
}

# --- Helper Functions ---
# Function to extract rain value
get_rain_value() {
    local extracted_value
    if [[ ! -f "$STATUS_FILE" ]] || [[ ! -r "$STATUS_FILE" ]]; then
        log_message "ERROR: Status file $STATUS_FILE not found or not readable."
        echo ""
        return 1
    fi

    if command -v jq > /dev/null; then
        extracted_value=$(jq -r '.rain' "$STATUS_FILE" 2>/dev/null)
        if [[ "$?" -ne 0 ]] || [[ "$extracted_value" == "null" ]] || [[ -z "$extracted_value" ]]; then
            echo ""
            return 1
        fi
    else
        extracted_value=$(grep '"rain"\s*:' "$STATUS_FILE" | sed -n 's/.*"rain"\s*:\s*\([0-9]\+\).*/\1/p')
        if [[ -z "$extracted_value" ]]; then
            echo ""
            return 1
        fi
    fi

    if [[ "$extracted_value" =~ ^[0-9]+$ ]]; then
        echo "$extracted_value"
        return 0
    else
        echo ""
        return 1
    fi
}

# Function to check dome status
check_dome_status() {
    local dome_status
    dome_status=$(curl -s --connect-timeout 5 --max-time 10 "http://$DOME_IP/\$S" 2>/dev/null | tr -d '\r\n\t ' | tr '[:lower:]' '[:upper:]')
    
    if [[ $? -ne 0 ]] || [[ -z "$dome_status" ]]; then
        log_message "ERROR: Could not retrieve dome status from $DOME_IP/\$S."
        echo "ERROR"
        return 1
    fi
    
    echo "$dome_status"
    return 0
}

# --- Main Script ---
log_message "Rain checker script started. Log file: $LOG_FILE"

while true; do
    RAIN_VALUE_STR=$(get_rain_value)
    
    if [[ $? -ne 0 ]] || [[ -z "$RAIN_VALUE_STR" ]]; then
        log_message "Warning: Could not extract valid rain value from $STATUS_FILE. Received value: '$RAIN_VALUE_STR'. Skipping cycle."
        sleep 10
        trim_log_file # Also trim log in case of error
        continue
    fi

    RAIN_VALUE=$((RAIN_VALUE_STR))
    log_message "Current rain value: $RAIN_VALUE (Rain threshold: < $RAIN_THRESHOLD)"

    if [[ "$RAIN_VALUE" -lt "$RAIN_THRESHOLD" ]]; then
        # Rain detected - PRIORITY: Rain sensor check
        log_message "RAIN DETECTED! Checking dome status..."
        
        # Check dome status
        DOME_STATUS=$(check_dome_status)
        if [[ "$DOME_STATUS" == "ERROR" ]]; then
            log_message "Warning: Dome status could not be retrieved. Performing ping test..."
            # Fallback to ping test if status query fails
        else
            log_message "Dome status: $DOME_STATUS"
            
            if [[ "$DOME_STATUS" == "CLOSE" ]] || [[ "$DOME_STATUS" == "CLOSED" ]]; then
                log_message "Dome is already closed. No action required."
                # Set rain flag if not already set
                if [[ ! -f "$RAIN_TRIGGERED" ]]; then
                    touch "$RAIN_TRIGGERED"
                    if [[ -f "$DRY_TRIGGERED" ]]; then
                        rm -f "$DRY_TRIGGERED"
                    fi
                fi
                sleep 10
                trim_log_file
                continue
            fi
        fi
        
        # Ping test (only if dome is not already closed or status unknown)
        if [[ ! -f "$PING_ALARM" ]]; then # Only check ping if no alarm is active or if it was successful
             if ! ping -c 1 "$DOME_IP" > /dev/null 2>&1; then
                log_message "Ping error to $DOME_IP detected – Sending ping alarm!"
                curl -s --form-string "token=$PUSHOVER_TOKEN" \
                         --form-string "user=$PUSHOVER_USER" \
                         --form-string "title=Ping Error (Rain active)" \
                         --form-string "message=Ping to $DOME_IP failed! Rain value: $RAIN_VALUE" \
                         --form-string "priority=1" \
                         "$PUSHOVER_API_URL" > /dev/null
                touch "$PING_ALARM"
            fi
        else # PING_ALARM exists
            if ping -c 1 "$DOME_IP" > /dev/null 2>&1; then
                 log_message "Ping to $DOME_IP successful again. Ping alarm is reset."
                 rm -f "$PING_ALARM"
            # else: Ping still failing, PING_ALARM remains
            fi
        fi
    
        if [[ ! -f "$RAIN_TRIGGERED" ]]; then
            log_message "RAAAAIN (Value: $RAIN_VALUE). Dome will be closed."
        
            curl -s --form-string "token=$PUSHOVER_TOKEN" \
                     --form-string "user=$PUSHOVER_USER" \
                     --form-string "title=RAAAAIN! (Value: $RAIN_VALUE)" \
                     --form-string "message=Dome closing!" \
                     --form-string "url=$PUSHOVER_INFO_URL" \
                     --form-string "priority=1" \
                     "$PUSHOVER_API_URL" > /dev/null

            log_message "WEST ($DOME_IP/\$3) closing!"
            curl -X GET -H "Content-Type: application/json" "http://$DOME_IP/\$3" >/dev/null
            sleep 3
            log_message "EAST ($DOME_IP/\$1) closing!"
            curl -X GET -H "Content-Type: application/json" "http://$DOME_IP/\$1" >/dev/null

            touch "$RAIN_TRIGGERED"
            if [[ -f "$DRY_TRIGGERED" ]]; then
                rm -f "$DRY_TRIGGERED"
            fi
            log_message "Rain mode activated. Waiting 5 minutes before next rain action (if flag is removed)."
            sleep 300
        else
            log_message "Rain already reported and dome action triggered (Value: $RAIN_VALUE). Waiting..."
        fi
    else
        # Dry
        if [[ -f "$RAIN_TRIGGERED" ]]; then
            log_message "Dry (Value: $RAIN_VALUE >= $RAIN_THRESHOLD). Rain status is cleared."
            rm -f "$RAIN_TRIGGERED"
        fi
        
        if [[ -f "$PING_ALARM" ]]; then
            log_message "Dry period: Ping alarm for $DOME_IP is reset."
            rm -f "$PING_ALARM"
        fi

        if [[ ! -f "$DRY_TRIGGERED" ]]; then
            log_message "DRYYYYY (Value: $RAIN_VALUE). Sending notification."
            curl -s --form-string "token=$PUSHOVER_TOKEN" \
                     --form-string "user=$PUSHOVER_USER" \
                     --form-string "title=DRYYYYY! (Value: $RAIN_VALUE)" \
                     --form-string "message=No more rain!" \
                     --form-string "url=$PUSHOVER_INFO_URL" \
                     --form-string "priority=1" \
                     "$PUSHOVER_API_URL" > /dev/null
            touch "$DRY_TRIGGERED"
        else
            # Already reported as dry, no action needed, just log for debugging if desired.
            # log_message "Dry status already active (Value: $RAIN_VALUE)."
            : # No-op, just continue
        fi
    fi
    
    sleep 10
    trim_log_file
done
