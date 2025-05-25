#!/bin/bash

# --- Configuration ---
#STATUS_FILE="/home/aagsolo/aag_json_test.dat"
STATUS_FILE="/home/aagsolo/aag_json.dat"
REGENTRIGGERED="/home/aagsolo/REGENTRIGGERED"
TROCKENTRIGGERED="/home/aagsolo/TROCKENTRIGGERED"
PINGALARM="/home/aagsolo/PINGALARM"

# Rain threshold: actions are triggered when the 'rain' value falls *below* this value.
RAIN_THRESHOLD=2000

# Log file configuration
LOG_FILE="/home/aagsolo/regenchecker.log"
MAX_LOG_LINES=5000

# Pushover configuration (for security reasons, better defined here rather than above)
PUSHOVER_TOKEN="xxxxxxxxxxxxxx"
PUSHOVER_USER="yyyyyyyyyyyyyyy"
PUSHOVER_API_URL="https://api.pushover.net/1/messages.json"
PUSHOVER_INFO_URL="http://94.xx.yy.zz:8888"

# Target IP for dome control
DOME_IP="192.168.1.177"

# --- Logging Functions ---
log_message() {
    local message="$1"
    local timestamped_message
    timestamped_message="$(date '+%Y-%m-%d %H:%M:%S'): $message"
    echo "$timestamped_message"  # Output to console
    echo "$timestamped_message" >> "$LOG_FILE"  # Append to log file
}

trim_log_file() {
    if [[ -f "$LOG_FILE" ]]; then
        local line_count
        line_count=$(wc -l < "$LOG_FILE" | tr -d ' ')  # Ensure only the number is read
        if (( line_count > MAX_LOG_LINES )); then
            log_message "Log file $LOG_FILE will be trimmed to the last $MAX_LOG_LINES lines (Current: $line_count lines)."
            local temp_log_file
            temp_log_file=$(mktemp "${LOG_FILE}.XXXXXX")  # Safe temporary filename
            if tail -n "$MAX_LOG_LINES" "$LOG_FILE" > "$temp_log_file"; then
                mv "$temp_log_file" "$LOG_FILE"
            else
                log_message "ERROR: Trimming log file $LOG_FILE failed."
                rm -f "$temp_log_file"  # Remove temp file on failure
            fi
        fi
    fi
}

# --- Helper Functions ---
# Function to extract the rain value
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

# --- Main Script ---
log_message "Rain checker script started. Log file: $LOG_FILE"

while true; do
    RAIN_VALUE_STR=$(get_rain_value)
    
    if [[ $? -ne 0 ]] || [[ -z "$RAIN_VALUE_STR" ]]; then
        log_message "Warning: Could not extract a valid rain value from $STATUS_FILE. Received: '$RAIN_VALUE_STR'. Skipping cycle."
        sleep 10
        trim_log_file  # Trim log even on error
        continue
    fi

    RAIN_VALUE=$((RAIN_VALUE_STR))
    log_message "Current rain value: $RAIN_VALUE (Threshold for rain: < $RAIN_THRESHOLD)"

    if [[ "$RAIN_VALUE" -lt "$RAIN_THRESHOLD" ]]; then
        # Rain detected
        if [[ ! -f "$PINGALARM" ]]; then  # Check ping only if no alarm active or if it has been cleared
             if ! ping -c 1 "$DOME_IP" > /dev/null 2>&1; then
                log_message "Ping failure to $DOME_IP detected – sending ping alarm!"
                curl -s --form-string "token=$PUSHOVER_TOKEN" \
                         --form-string "user=$PUSHOVER_USER" \
                         --form-string "title=Ping Failure (Rain active)" \
                         --form-string "message=Ping to $DOME_IP failed! Rain value: $RAIN_VALUE" \
                         --form-string "priority=1" \
                         "$PUSHOVER_API_URL" > /dev/null
                touch "$PINGALARM"
            fi
        else  # PINGALARM exists
            if ping -c 1 "$DOME_IP" > /dev/null 2>&1; then
                 log_message "Ping to $DOME_IP successful again. Clearing ping alarm."
                 rm -f "$PINGALARM"
            fi  # else: still failing, leave PINGALARM in place
        fi
    
        if [[ ! -f "$REGENTRIGGERED" ]]; then
            log_message "RAIN DETECTED (Value: $RAIN_VALUE). Closing dome."
        
            curl -s --form-string "token=$PUSHOVER_TOKEN" \
                     --form-string "user=$PUSHOVER_USER" \
                     --form-string "title=RAIN! (Value: $RAIN_VALUE)" \
                     --form-string "message=Closing dome!" \
                     --form-string "url=$PUSHOVER_INFO_URL" \
                     --form-string "priority=1" \
                     "$PUSHOVER_API_URL" > /dev/null

            log_message "WEST ($DOME_IP/\$3) closing!"
            curl -X GET -H "Content-Type: application/json" "http://$DOME_IP/\$3" >/dev/null
            sleep 3
            log_message "EAST ($DOME_IP/\$1) closing!"
            curl -X GET -H "Content-Type: application/json" "http://$DOME_IP/\$1" >/dev/null

            touch "$REGENTRIGGERED"
            if [[ -f "$TROCKENTRIGGERED" ]]; then
                rm -f "$TROCKENTRIGGERED"
            fi
            log_message "Rain mode activated. Waiting 5 minutes before next rain action (in case flag is removed)."
            sleep 300
        else
            log_message "Rain already reported and dome action performed (Value: $RAIN_VALUE). Waiting..."
        fi
    else
        # Dry condition
        if [[ -f "$REGENTRIGGERED" ]]; then
            log_message "Dry (Value: $RAIN_VALUE >= $RAIN_THRESHOLD). Clearing rain status."
            rm -f "$REGENTRIGGERED"
        fi
        
        if [[ -f "$PINGALARM" ]]; then
            log_message "Dry period: Clearing ping alarm for $DOME_IP."
            rm -f "$PINGALARM"
        fi

        if [[ ! -f "$TROCKENTRIGGERED" ]]; then
            log_message "DRY! (Value: $RAIN_VALUE). Sending notification."
            curl -s --form-string "token=$PUSHOVER_TOKEN" \
                     --form-string "user=$PUSHOVER_USER" \
                     --form-string "title=DRY! (Value: $RAIN_VALUE)" \
                     --form-string "message=No more rain!" \
                     --form-string "url=$PUSHOVER_INFO_URL" \
                     --form-string "priority=1" \  # Retaining priority 1 as in original
                     "$PUSHOVER_API_URL" > /dev/null
            touch "$TROCKENTRIGGERED"
        else
            # Already reported dry status, no action needed. Logging only for debugging if desired.
            :  # No-op, continue
        fi
    fi
    
    sleep 10
    trim_log_file
done
