#!/bin/bash
# Setup script for VYRA Monitoring Stack (Prometheus + Grafana)
# Located in: vyra_vizu/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==================================="
echo "VYRA Monitoring Stack Setup"
echo "Location: vyra_vizu/"
echo "==================================="

# Check if configs exist
if [ ! -f "prometheus/prometheus.yml" ]; then
    echo "❌ prometheus/prometheus.yml missing!"
    exit 1
fi

if [ ! -f "grafana/provisioning/datasources/prometheus.yml" ]; then
    echo "❌ grafana/provisioning/datasources/prometheus.yml missing!"
    exit 1
fi

echo "✓ Configuration files found"

# Check if docker-compose.monitoring.yml exists
if [ ! -f "docker-compose.monitoring.yml" ]; then
    echo "❌ docker-compose.monitoring.yml missing!"
    exit 1
fi

echo "✓ Docker Compose configuration validated"

# Deploy monitoring services
echo ""
echo "Deploying monitoring stack..."
docker stack deploy -c docker-compose.monitoring.yml vos2_ws

echo ""
echo "==================================="
echo "✓ Monitoring stack deployed!"
echo "==================================="
echo ""
echo "Services:"
echo "  Prometheus: http://localhost:9090"
echo "  Grafana:    http://localhost:3001"
echo "              https://localhost/grafana/"
echo ""
echo "Grafana Login:"
echo "  Username: admin"
echo "  Password: vyra_admin_2024"
echo ""
echo "Checking service status in 10 seconds..."
sleep 10

# Check services
echo ""
echo "Service status:"
docker service ls | grep -E "prometheus|grafana" || true

echo ""
echo "Container status:"
docker ps | grep -E "prometheus|grafana" || true

echo ""
echo "✓ Setup complete!"
echo ""
echo "Next steps:"
echo "  1. Open Grafana: http://localhost:3001"
echo "  2. Login with admin/vyra_admin_2024"
echo "  3. Create dashboards with Prometheus queries:"
echo "     - vyra_module_cpu_percent"
echo "     - vyra_module_memory_mb"
echo "     - vyra_modules_total"
