#!/bin/bash
cd website
npm install
npm run build

# Copy the built files to the root directory so Vercel can serve them
cp -r build/* ../ 2>/dev/null || xcopy /E /I build\\* ..\\ 2>nul