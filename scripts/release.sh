#!/bin/bash

# Script to help with the release process
# リリースプロセスを支援するスクリプト

if [ -z "$1" ]; then
  echo "Error: No version specified"
  echo "Usage: ./scripts/release.sh <version|patch|minor|major>"
  exit 1
fi

VERSION_ARG=$1

if [[ "$VERSION_ARG" =~ ^(patch|minor|major)$ ]]; then
  INCREMENT_TYPE=$VERSION_ARG
  CURRENT_VERSION=$(grep -o '^version = "[^"]*"' pyproject.toml | cut -d'"' -f2)
  echo "Using semantic increment: $INCREMENT_TYPE (current version: $CURRENT_VERSION)"
else
  if ! [[ $VERSION_ARG =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z-]+)?(\+[0-9A-Za-z-]+)?$ ]]; then
    echo "Error: Version must follow semantic versioning (e.g., 1.2.3, 1.2.3-beta, etc.)"
    echo "Or use one of: patch, minor, major"
    exit 1
  fi
  SPECIFIC_VERSION=$VERSION_ARG
  echo "Using specific version: $SPECIFIC_VERSION"
fi

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT_BRANCH" != "main" ]; then
  echo "Warning: You are not on the main branch. Current branch: $CURRENT_BRANCH"
  read -p "Do you want to continue? (y/n) " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
  fi
fi

if [ -n "$(git status --porcelain)" ]; then
  echo "Error: Working directory is not clean. Please commit or stash your changes."
  exit 1
fi

echo "Pulling latest changes from origin..."
git pull origin main

PACKAGE_VERSION=$(grep -o '^version = "[^"]*"' pyproject.toml | cut -d'"' -f2)
LOCK_VERSION=$(grep -A 1 'name = "takanarishimbo-ros2-exec-mcp"' uv.lock | grep 'version = ' | cut -d'"' -f2)
SERVER_VERSION=$(grep -o '__version__ = "[^"]*"' src/__init__.py | cut -d'"' -f2)

echo "Current versions:"
echo "- pyproject.toml: $PACKAGE_VERSION"
echo "- uv.lock: $LOCK_VERSION"
echo "- server: $SERVER_VERSION"

update_server_version() {
  local old_version=$1
  local new_version=$2
  local commit_msg=$3
  
  echo "Updating version in __init__.py from $old_version to $new_version..."
  if [[ "$OSTYPE" == "darwin"* ]]; then
    sed -i '' "s/__version__ = \"$old_version\"/__version__ = \"$new_version\"/" src/__init__.py
  else
    sed -i "s/__version__ = \"$old_version\"/__version__ = \"$new_version\"/" src/__init__.py
  fi
  git add src/__init__.py
  
  if [ -n "$commit_msg" ]; then
    git commit -m "$commit_msg"
  fi
}

if [ "$PACKAGE_VERSION" != "$LOCK_VERSION" ] || [ "$PACKAGE_VERSION" != "$SERVER_VERSION" ]; then
  echo "Warning: Version mismatch detected between files."
  if [ -n "$SPECIFIC_VERSION" ]; then
    echo "Will update all files to version: $SPECIFIC_VERSION"
  else
    echo "Will update all files using increment: $INCREMENT_TYPE"
  fi
  read -p "Do you want to continue? (y/n) " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 1
  fi
fi

if [ -n "$INCREMENT_TYPE" ]; then
  echo "Incrementing version ($INCREMENT_TYPE)..."
  CURRENT_VERSION=$(grep -o '^version = "[^"]*"' pyproject.toml | cut -d'"' -f2)
  IFS='.' read -r major minor patch <<< "$CURRENT_VERSION"
  if [ "$INCREMENT_TYPE" = "major" ]; then
    major=$((major + 1)); minor=0; patch=0
  elif [ "$INCREMENT_TYPE" = "minor" ]; then
    minor=$((minor + 1)); patch=0
  elif [ "$INCREMENT_TYPE" = "patch" ]; then
    patch=$((patch + 1))
  fi
  NEW_VERSION="$major.$minor.$patch"
  if [[ "$OSTYPE" == "darwin"* ]]; then
    sed -i '' "s/version = \"$CURRENT_VERSION\"/version = \"$NEW_VERSION\"/" pyproject.toml
  else
    sed -i "s/version = \"$CURRENT_VERSION\"/version = \"$NEW_VERSION\"/" pyproject.toml
  fi
  update_server_version "$SERVER_VERSION" "$NEW_VERSION"
  echo "Cleaning up old virtual environment..."
  rm -rf .venv
  uv lock
  git add pyproject.toml uv.lock
  git commit -m "chore: release version $NEW_VERSION"
  git tag -a "v$NEW_VERSION" -m "Version $NEW_VERSION"
else
  echo "Updating version in pyproject.toml and uv.lock..."
  CURRENT_VERSION=$(grep -o '^version = "[^"]*"' pyproject.toml | cut -d'"' -f2)
  if [[ "$OSTYPE" == "darwin"* ]]; then
    sed -i '' "s/version = \"$CURRENT_VERSION\"/version = \"$SPECIFIC_VERSION\"/" pyproject.toml
  else
    sed -i "s/version = \"$CURRENT_VERSION\"/version = \"$SPECIFIC_VERSION\"/" pyproject.toml
  fi
  update_server_version "$SERVER_VERSION" "$SPECIFIC_VERSION"
  echo "Cleaning up old virtual environment..."
  rm -rf .venv
  uv lock
  git add pyproject.toml uv.lock
  git commit -m "chore: release version $SPECIFIC_VERSION"
  git tag -a "v$SPECIFIC_VERSION" -m "Version $SPECIFIC_VERSION"
fi

FINAL_VERSION=$(grep -o '^version = "[^"]*"' pyproject.toml | cut -d'"' -f2)
echo "Pushing changes and tag to remote..."
git push origin main
git push origin v$FINAL_VERSION

echo "Release process completed for version $FINAL_VERSION"
echo "The GitHub workflow will now build and publish the package to PyPI"
echo "Check the Actions tab in your GitHub repository for progress"

