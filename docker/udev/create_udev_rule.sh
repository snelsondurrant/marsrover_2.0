#!/bin/bash

# Function to check if required packages are installed
check_packages() {
    for package in "$@"; do
        if ! dpkg -l | grep -q "$package"; then
            echo "Package '$package' is not installed. You can install it using:"
            echo "sudo apt install $package"
            exit 1
        fi
    done
}

# Check for required packages
check_packages "usbutils"

# Check devices before plugging in
echo "Please make sure the device is unplugged."
echo " "
read -n 1 -s -r -p "Press any key once the device is unplugged and ready to plug in..."

# Get initial device list
INITIAL_DEVICES=$(lsusb)

# Prompt to plug in the device
echo " "
echo "Now, please plug in the device you want to create a udev rule for."
echo " "
read -n 1 -s -r -p "Press any key once the device is plugged in..."

# Get the new device list
FINAL_DEVICES=$(lsusb)

# Compare device lists
NEW_DEVICE_INFO=$(diff <(echo "$INITIAL_DEVICES") <(echo "$FINAL_DEVICES") | grep '>' | sed 's/^> //')

if [ -z "$NEW_DEVICE_INFO" ]; then
    echo " "
    echo "No new USB device detected. Please try again."
    exit 1
fi

echo "Detected new device: $NEW_DEVICE_INFO"

# Check dmesg for the device path
echo " "
echo "Checking dmesg for the new device path..."
dmesg_output=$(sudo dmesg | tail -n 20)
DEV_PATH=$(echo "$dmesg_output" | grep -oP 'ttyUSB[0-9]+' || echo "$dmesg_output" | grep -oP 'ttyACM[0-9]+' | tail -n 1)

if [ -z "$DEV_PATH" ]; then
    echo "No ttyUSB or ttyACM device found in dmesg. Please ensure your device is connected correctly."
    exit 1
fi

# Ask for user confirmation on the identified device path
echo " "
echo "Device path identified: $DEV_PATH (without /dev prefix)"
read -p "Does this look like the correct path? (y/n): " CONFIRM

if [[ "$CONFIRM" != "y" ]]; then
    echo "Please reconnect your device and try again."
    exit 1
fi

# Get detailed info about the new device
udevadm_info=$(udevadm info --query=all --name="/dev/$DEV_PATH")

# Extract necessary attributes
VENDOR_ID=$(echo "$udevadm_info" | grep "ID_VENDOR_ID=" | cut -d'=' -f2)
PRODUCT_ID=$(echo "$udevadm_info" | grep "ID_MODEL_ID=" | cut -d'=' -f2)
ID_SERIAL=$(echo "$udevadm_info" | grep "ID_SERIAL=" | cut -d'=' -f2)
ID_VENDOR=$(echo "$udevadm_info" | grep "ID_VENDOR=" | cut -d'=' -f2)
ID_MODEL=$(echo "$udevadm_info" | grep "ID_MODEL=" | cut -d'=' -f2)

echo "Vendor ID: $VENDOR_ID"
echo "Product ID: $PRODUCT_ID"
echo "Serial: $ID_SERIAL"
echo "Vendor: $ID_VENDOR"
echo "Model: $ID_MODEL"

# Prompt user for additional attributes
echo "Select additional attributes to identify the device (press enter after each selection):"
echo "1. Use ATTRS for vendor and product IDs only"
echo "2. Use ATTRS for vendor, product, and serial"
echo "3. Use ATTR for vendor and product IDs only"
echo "4. Use ATTR for vendor, product, and serial"
read -p "Enter your selection (1, 2, 3, 4): " SELECTION

# Initialize rule components
RULE_COMPONENTS=("SUBSYSTEM==\"tty\"")

# Add attributes based on selection
case "$SELECTION" in
    4)
        RULE_COMPONENTS+=(" ATTR{idVendor}==\"$VENDOR_ID\"")
        RULE_COMPONENTS+=(" ATTR{idProduct}==\"$PRODUCT_ID\"")
        RULE_COMPONENTS+=(" ATTR{serial}==\"$ID_SERIAL\"")
        ;;
    3)
        RULE_COMPONENTS+=(" ATTR{idVendor}==\"$VENDOR_ID\"")
        RULE_COMPONENTS+=(" ATTR{idProduct}==\"$PRODUCT_ID\"")
        ;;
    2)
        RULE_COMPONENTS+=(" ATTRS{idVendor}==\"$VENDOR_ID\"")
        RULE_COMPONENTS+=(" ATTRS{idProduct}==\"$PRODUCT_ID\"")
        RULE_COMPONENTS+=(" ATTRS{serial}==\"$ID_SERIAL\"")
        ;;
    1)
        RULE_COMPONENTS+=(" ATTRS{idVendor}==\"$VENDOR_ID\"")
        RULE_COMPONENTS+=(" ATTRS{idProduct}==\"$PRODUCT_ID\"")
        ;;
    *)
        echo "Invalid selection: $SELECTION"
        ;;
esac

# Ask for symlink path
read -p "Enter the desired symlink path for the device (e.g., rover/mydevice): " SYMLINK_PATH

# Construct the final rule with the symlink
RULE_TEXT=$(IFS=','; echo "${RULE_COMPONENTS[*]}, SYMLINK+=\"$SYMLINK_PATH\"")

# Output the rule text with highlighting
echo -e "\n=============================="
echo -e "\033[1;32mHere is the udev rule you can use:\033[0m"
echo -e "\033[1;34m$RULE_TEXT\033[0m"
echo -e "==============================\n"

# Provide guidance on what to do next
echo "To create a new rule file, follow these steps:"
echo "1. Open a terminal and run:"
echo "   sudo nano /etc/udev/rules.d/99-usb-device.rules"
echo "2. Paste the rule you just generated."
echo "3. Save the file and exit (Ctrl + O, Enter, Ctrl + X in nano)."
echo "4. Run the following command to apply the changes:"
echo "   sudo udevadm control --reload-rules"
echo "5. Finally, disconnect and reconnect the device for the rule to take effect."
