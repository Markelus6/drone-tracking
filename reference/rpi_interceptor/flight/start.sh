#!/bin/sh
# Make this file executable, so that it will be executed during the boot process as root.

# Uncomment to create a backup of the encryption key and unlock program.
# cp /etc/fscrypt.key /opt/data/fscrypt.key
# cp /usr/local/bin/unlock /opt/data/unlock

# Uncomment to securely delete the encryption key and unlock program.
# wipe -sf /etc/fscrypt.key
# wipe -sf /usr/local/bin/unlock

while true; do
if (/usr/bin/env --chdir=/opt/data/copters_onboard/build ./app != 0) then
break
fi
done
