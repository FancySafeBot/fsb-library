#!/bin/bash
set -e

# Create directory for PlantUML
INSTALL_DIR="/opt/plantuml"
sudo mkdir -p "$INSTALL_DIR"

# Download latest PlantUML jar
PLANTUML_URL="https://github.com/plantuml/plantuml/releases/latest/download/plantuml.jar"
sudo wget -O "$INSTALL_DIR/plantuml.jar" "$PLANTUML_URL"

# Create a symlink to /usr/local/bin
sudo ln -sf "$INSTALL_DIR/plantuml.jar" /usr/local/bin/plantuml.jar

# Create a wrapper script for easy execution
sudo tee /usr/local/bin/plantuml > /dev/null <<'EOF'
#!/bin/bash
exec java -jar /usr/local/bin/plantuml.jar "$@"
EOF
sudo chmod +x /usr/local/bin/plantuml

echo "PlantUML installed. Run with: plantuml <file>"
