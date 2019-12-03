#!/bin/sh

# Decrypt the file
# --batch to prevent interactive command --yes to assume "yes" for questions
gpg --quiet --batch --yes --decrypt --passphrase="$MJPASSPHRASE" \
--output test/mjkey.txt test/mjkey.txt.gpg 
