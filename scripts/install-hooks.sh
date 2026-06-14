#!/bin/bash
#
# Install git hooks for d2x-cios
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HOOKS_DIR="$REPO_ROOT/.git/hooks"

echo "Installing git hooks for d2x-cios..."

# Check if we're in a git repository
if [ ! -d "$REPO_ROOT/.git" ]; then
    echo "ERROR: Not in a git repository"
    exit 1
fi

# Install pre-commit hook
echo "  Installing pre-commit hook..."
cp "$SCRIPT_DIR/hooks/pre-commit" "$HOOKS_DIR/pre-commit"
chmod +x "$HOOKS_DIR/pre-commit"

echo ""
echo "âœ“ Git hooks installed successfully!"
echo ""
echo "The pre-commit hook will:"
echo "  - Verify that d2x-cios builds successfully before each commit"
echo "  - Auto-build devkitarm-r32 Docker image if needed (one-time ~15-30 min)"
echo "  - Use cached Docker image for fast subsequent builds"
echo ""
echo "To skip the hook for a specific commit, use: git commit --no-verify"
echo ""
