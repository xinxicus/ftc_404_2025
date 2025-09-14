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
