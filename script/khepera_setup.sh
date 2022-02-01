#!/bin/sh
# Created by Indraneel on 26/01/21

CODE_PATH=$1
KHEPERA_IP_ADDRESS=$2
SERVER_IP_ADDRESS=$3
FEEDBACK_PORT=$4
CONTROL_PORT=$5
FEEDBACK_FREQUENCY=$6
CONTROL_TIMEOUT=$7


# Kill any old running code
ssh root@${KHEPERA_IP_ADDRESS} pkill template

# Transfer latest khepera code
scp ${CODE_PATH} root@${KHEPERA_IP_ADDRESS}:/home/root

# Execute latest khepera code with the right arguments
ssh root@${KHEPERA_IP_ADDRESS} ./template ${SERVER_IP_ADDRESS} ${CONTROL_PORT} ${FEEDBACK_PORT} ${FEEDBACK_FREQUENCY} ${CONTROL_TIMEOUT} &
