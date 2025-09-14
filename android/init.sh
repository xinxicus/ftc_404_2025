#!/bin/bash

# FTC Development Environment Initialization Script
# Sets up clean separation: ftcSDK (OOTB) + teamCode (your work)

echo "🚀 Initializing FTC Development Environment..."

# Check if ftcSDK exists, if not clone it
if [ ! -d "ftcSDK" ]; then
    echo "📥 ftcSDK not found, cloning from official repository..."
    git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git ftcSDK
    echo "✅ ftcSDK cloned successfully"
else
    echo "📚 ftcSDK already exists"
fi

# Create teamCode directory structure if it doesn't exist
if [ ! -d "teamCode/src/main/java/org/firstinspires/ftc/teamcode" ]; then
    echo "📁 Creating teamCode directory structure..."
    mkdir -p teamCode/src/main/java/org/firstinspires/ftc/teamcode
    echo "✅ teamCode directory structure created"
fi

# Create symlink from teamCode to ftcSDK TeamCode
TEAMCODE_LINK="teamCode/src/main/java/org/firstinspires/ftc/teamcode/error404"
FTCSDK_TEAMCODE="ftcSDK/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/error404"

# Remove existing symlink or directory if it exists
if [ -L "$TEAMCODE_LINK" ] || [ -d "$TEAMCODE_LINK" ]; then
    echo "🔗 Removing existing symlink/directory..."
    rm -rf "$TEAMCODE_LINK"
fi

# Create new symlink
echo "🔗 Creating symlink from teamCode to ftcSDK..."
ln -s "../../../../../../../../$FTCSDK_TEAMCODE" "$TEAMCODE_LINK"
echo "✅ Symlink created"

# Copy local.properties to ftcSDK if it doesn't exist
if [ ! -f "ftcSDK/local.properties" ] && [ -f "local.properties" ]; then
    echo "📋 Copying local.properties to ftcSDK..."
    cp local.properties ftcSDK/
    echo "✅ local.properties copied"
fi

# Apply custom dependencies from android project using apply from
if [ -f "custom.dependencies.gradle" ]; then
    echo "📦 Applying custom dependencies using apply from..."
    
    # Create a backup of the original file
    cp ftcSDK/build.dependencies.gradle ftcSDK/build.dependencies.gradle.backup
    
    # Add apply from statement at the end of the file
    echo "" >> ftcSDK/build.dependencies.gradle
    echo "// Custom dependencies from android project" >> ftcSDK/build.dependencies.gradle
    echo "apply from: '${PWD}/custom.dependencies.gradle'" >> ftcSDK/build.dependencies.gradle
    
    echo "✅ Custom dependencies applied using apply from"
else
    echo "⚠️  No custom.dependencies.gradle found in android project"
fi


# Create simplified build script
echo "🔨 Creating simplified build script..."
cat > build.sh << 'EOF'
#!/bin/bash

echo "🔧 Building FTC Robot Controller..."

# Build using ftcSDK directly
ftcSDK/gradlew -p ftcSDK :TeamCode:assembleDebug

# Copy entire output folder to project root
echo "📦 Copying build outputs..."
cp -rf ftcSDK/TeamCode/build/outputs/apk/debug ./output

echo "✅ Build complete!"
echo "📱 APK available at: output/TeamCode-debug.apk"
echo "📁 Build outputs available in: output/"
EOF

chmod +x build.sh

echo ""
echo "✅ Initialization complete!"
echo ""
echo "🎯 Next steps:"
echo "   1. Add your OpModes to: teamCode/src/main/java/org/firstinspires/ftc/teamcode/error404/"
echo "   2. Run ./build.sh to build your APK"
echo ""
echo "📁 Project structure:"
echo "   ftcSDK/                    # OOTB SDK"
echo "   teamCode/                  # Your work (symlinked to ftcSDK)"