# QuickPush.ps1 - Quick add / commit / push

# 1. Set your actual project path
$ProjectDir = "D:\Express_pickup_point"

# 2. Switch to the project directory
if (-not (Test-Path -Path $ProjectDir)) {
    Write-Output "Unable to enter directory: $ProjectDir"
    exit 1
}

Set-Location -Path $ProjectDir

git pull origin main
# 3. Add all changes
git add .

# 4. Commit changes
git commit -m "null" --allow-empty

# 5. Push to the remote main branch
git push origin master