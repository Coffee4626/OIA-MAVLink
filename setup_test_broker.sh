#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
CERT_DIR="$PROJECT_DIR/certs"
MQTT_USER="drone01"
MQTT_PASS="password"
CN="localhost"

echo "installing mosquitto broker"

sudo apt-get update
sudo apt-get install -y --no-install-recommends mosquitto mosquitto-clients openssl

echo ""
echo "generating self-signed TLS certs ($CERT_DIR)"
mkdir -p "$CERT_DIR"; cd "$CERT_DIR"

openssl genrsa -out ca.key 2048
openssl req -new -x509 -days 365 -key ca.key -out ca.crt -subj "/CN=OIA Local CA"
openssl genrsa -out server.key 2048
openssl req -new -key server.key -out server.csr -subj "/CN=$CN"
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial \
    -out server.crt -days 365
rm -f server.csr

echo "generated ca.crt, server.crt, server.key"
echo ""

cd "$PROJECT_DIR"
echo ""

echo "installing certs for broker"
sudo mkdir -p /etc/mosquitto/certs
sudo cp "$CERT_DIR"/{ca.crt,server.crt,server.key} /etc/mosquitto/certs/
sudo chown mosquitto:mosquitto /etc/mosquitto/certs/*
sudo chmod 640 /etc/mosquitto/certs/*

echo ""
echo "configuring auth and tls"
sudo mosquitto_passwd -b -c /etc/mosquitto/passwd "$MQTT_USER" "$MQTT_PASS"
sudo chown mosquitto:mosquitto /etc/mosquitto/passwd
sudo chmod 640 /etc/mosquitto/passwd
sudo tee /etc/mosquitto/conf.d/drone.conf > /dev/null << CONF
listener 8883
cafile   /etc/mosquitto/certs/ca.crt
certfile /etc/mosquitto/certs/server.crt
keyfile  /etc/mosquitto/certs/server.key
allow_anonymous false
password_file /etc/mosquitto/passwd
CONF

if ! grep -q "include_dir /etc/mosquitto/conf.d" /etc/mosquitto/mosquitto.conf; then
    echo "include_dir /etc/mosquitto/conf.d" | sudo tee -a /etc/mosquitto/mosquitto.conf > /dev/null
fi

sudo systemctl restart mosquitto
sleep 10
if sudo ss -tlnp | grep -q 8883; then
    echo "    broker listening on 8883 OK"
else
    echo "    WARNING: broker not on 8883, check: sudo journalctl -u mosquitto -n 20"
fi

echo ""
echo "finished setting up local broker for testing"
echo "config.conf should use: broker_ip=$CN, broker_port=8883,"
echo "    ca_cert_path=$CERT_DIR/ca.crt, username=$MQTT_USER, password=$MQTT_PASS"
echo ""
echo "Watch telemetry:"
echo "    mosquitto_sub -h $CN -p 8883 --cafile $CERT_DIR/ca.crt \\"
echo "        -u $MQTT_USER -P '$MQTT_PASS' -t 'drone/#' -v"
