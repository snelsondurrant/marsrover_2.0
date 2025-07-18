#!/bin/bash
# Created by Braden Meyers & Nelson Durrant, Mar 2025
#
# Set up a SSH key for passwordless access to the Mars Rover

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

remove_old_ssh_key() {
    # Removes an old SSH key from known_hosts
    local host=$1
    local port=$2

    if [[ "$port" == "22" ]]; then
        # Remove without specifying port notation for default SSH port
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$host" &> /dev/null
    else
        # Remove using port notation for non-default ports
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[$host]:$port" &> /dev/null
    fi
}

if ! command -v sshpass &> /dev/null; then
    printWarning "Please install 'sshpass' using 'sudo apt install sshpass'"
    ROVER_CONNECT="ssh"
else
    ROVER_CONNECT="sshpass -p $ROVER_PASSWORD ssh"
fi

remove_old_ssh_key "$ROVER_IP_ADDRESS" "22"
$ROVER_CONNECT -o StrictHostKeyChecking=accept-new $ROVER_USERNAME@$ROVER_IP_ADDRESS "echo" &> /dev/null
ssh-copy-id $ROVER_USERNAME@$ROVER_IP_ADDRESS
