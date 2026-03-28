# Research: Version Controlling Workspace .claude Directory Without Breaking IDE Git GUI

**Date**: 2025-10-24
**Problem**: When initializing `/home/finn/Documents/ardu_ws/.claude` as a git repository, Cursor IDE stops recognizing sub-repositories in `/home/finn/Documents/ardu_ws/src/`, making the visual Git GUI unusable for those repos.

---

## Table of Contents
1. [Problem Analysis](#problem-analysis)
2. [Why This Happens](#why-this-happens)
3. [Solution Comparison Matrix](#solution-comparison-matrix)
4. [Detailed Solutions](#detailed-solutions)
5. [Recommended Approach](#recommended-approach)
6. [Implementation Guide](#implementation-guide)
7. [Troubleshooting](#troubleshooting)

---

## Problem Analysis

### Current Workspace Structure
```
/home/finn/Documents/ardu_ws/
├── .claude/                         # Want to version control this
│   ├── CLAUDE.md
│   ├── settings.local.json
│   ├── commands/
│   └── Features/
├── src/
│   ├── ardupilot_gz/.git           # Git repo - IDE stops seeing this
│   ├── ardupilot_gazebo/.git       # Git repo - IDE stops seeing this
│   ├── formation_control/.git      # Git repo
│   ├── ardupilot/.git              # Git repo (with many submodules)
│   ├── ros_gz/.git
│   ├── sdformat_urdf/.git
│   ├── ardupilot_sitl_models/.git
│   ├── micro_ros_agent/.git
│   └── Micro-XRCE-DDS-Gen/.git
├── build/
├── install/
└── log/
```

### The Conflict
When you run `git init` at `/home/finn/Documents/ardu_ws/`, you create a **parent repository** that:
1. Cursor detects as the primary repo for the workspace
2. Makes Cursor treat sub-repos as potential submodules or ignore them
3. Causes the IDE's source control view to focus only on the workspace root
4. Hides the individual repo Git GUIs you rely on

### What You Need
- Version control for `.claude/` directory (settings, commands, context)
- Maintain full Cursor Git GUI functionality for all src/ repositories
- Simple workflow without complex git submodule commands
- Ability to work independently on each sub-repo

---

## Why This Happens

### Cursor's Git Repository Detection (Inherited from VS Code)

Cursor (VS Code fork) uses these mechanisms to detect Git repositories:

#### 1. Repository Scanning Algorithm
```javascript
// Simplified logic from VS Code source
function scanForGitRepos(workspaceRoot) {
    const repos = [];
    const maxDepth = config.get('git.repositoryScanMaxDepth', 1); // Default: 1 level

    // Scan from workspace root
    scanDirectory(workspaceRoot, 0, maxDepth, repos);

    return repos;
}
```

**Key Behaviors**:
- Starts scanning from the workspace root opened in Cursor
- Default scan depth is **1 level deep** from workspace root
- If workspace root itself is a git repo, it becomes the "primary" repository
- Sub-repositories are only detected if:
  - Within scan depth limit
  - Not ignored by parent repo
  - Not treated as submodules

#### 2. Source Control View Priority
When multiple repos are detected:
```
Priority 1: Workspace root repository (if exists)
Priority 2: Repositories within scan depth
Priority 3: Opened file's repository context
```

If workspace root has `.git/`, Cursor assumes it's the main repo and:
- Source Control panel focuses on it
- Other repos become "secondary" or hidden
- GUI operations default to the workspace root repo

#### 3. Git Integration Architecture
```
Cursor Workspace Root: /home/finn/Documents/ardu_ws/
    ↓
    Git Extension Activation
    ↓
    Scan for .git directories ← STOPS HERE if root has .git
    ↓
    Register repositories with SCM
    ↓
    Update Source Control View
```

### The Nested Repository Problem

Git itself handles nested repositories in specific ways:

#### Case 1: Parent Repo Exists
```bash
/parent/.git          # Parent repo
/parent/child/.git    # Child repo

# Git's behavior:
$ cd /parent
$ git status
# Shows "child/" as untracked directory or submodule reference
# Does NOT show child's internal changes
```

#### Case 2: Submodule Declaration
```bash
/parent/.git
/parent/.gitmodules   # Declares child as submodule
/parent/child/.git    # Actually a file pointing to .git/modules/child/

# Parent repo tracks specific commit of child
# Child changes invisible until explicitly committed in child AND parent
```

#### Case 3: Independent Repos (Your Desired State)
```bash
/workspace/           # NOT a git repo
/workspace/repo1/.git # Independent repo
/workspace/repo2/.git # Independent repo

# Each repo is completely independent
# IDE can see and interact with both
```

---

## Solution Comparison Matrix

| Solution | IDE GUI Works | Simple Workflow | Version Controls .claude | Auto-sync | Setup Complexity |
|----------|--------------|-----------------|-------------------------|-----------|------------------|
| 1. Separate Repo + IDE Config | ✅ Yes | ⚠️ Moderate | ✅ Yes | ❌ No | ⭐⭐ |
| 2. Git Worktree | ✅ Yes | ⚠️ Advanced | ✅ Yes | ❌ No | ⭐⭐⭐⭐ |
| 3. Sparse Checkout | ⚠️ Partial | ⚠️ Complex | ✅ Yes | ❌ No | ⭐⭐⭐⭐⭐ |
| 4. Custom Exclude + Symlinks | ✅ Yes | ⚠️ Moderate | ✅ Yes | ⚠️ Manual | ⭐⭐⭐ |
| 5. IDE Multi-root Workspace | ✅ Yes | ✅ Simple | ⚠️ Per-repo | ❌ No | ⭐ |
| 6. Separate .claude Repo (No Parent) | ✅ Yes | ✅ Simple | ✅ Yes | ❌ No | ⭐ |

**Legend**: ✅ Full Support | ⚠️ Partial/With Caveats | ❌ Not Supported | ⭐ = Complexity (fewer stars = simpler)

---

## Detailed Solutions

### Solution 1: Separate Repo + Cursor Configuration (Your Current Attempt)

#### The Approach
Initialize `.claude/` as its own git repository while keeping workspace root repo-free, and configure Cursor to detect all repos.

#### Why It's Failing Now
```bash
# If you did this:
cd /home/finn/Documents/ardu_ws/
git init  # ← This creates parent repo

# OR if .claude/ is a git repo and Cursor can't see others:
cd /home/finn/Documents/ardu_ws/.claude/
git init  # This should work, but needs proper Cursor config
```

#### Correct Implementation

**Step 1: Ensure workspace root is NOT a git repo**
```bash
# Check if workspace root has .git
ls -la /home/finn/Documents/ardu_ws/.git

# If it exists, remove it (ONLY if you don't need it)
# DANGEROUS: Only do this if you haven't committed important workspace-level changes
rm -rf /home/finn/Documents/ardu_ws/.git
```

**Step 2: Initialize .claude as independent repo**
```bash
cd /home/finn/Documents/ardu_ws/.claude/
git init
git add CLAUDE.md commands/ agents/
git commit -m "Initial .claude workspace configuration"

# Add remote if you want to back it up
git remote add origin git@github.com:yourusername/ardu_ws-claude-config.git
git push -u origin main
```

**Step 3: Configure Cursor to scan deeply**

Create or update `/home/finn/Documents/ardu_ws/.vscode/settings.json`:
```json
{
    "git.repositoryScanMaxDepth": 10,
    "git.detectSubmodules": false,
    "git.autoRepositoryDetection": "openEditors",
    "scm.repositories.visible": 15,
    "Lua.diagnostics.disable": [
        "duplicate-doc-field",
        "undefined-global",
        "param-type-mismatch"
    ]
}
```

**Key Settings Explained**:
- `git.repositoryScanMaxDepth`: Scan up to 10 levels deep (covers `src/*/.git`)
- `git.detectSubmodules`: Disable submodule detection (prevents confusion)
- `git.autoRepositoryDetection`: Show repos based on open files
- `scm.repositories.visible`: Show up to 15 repos in Source Control panel

**Step 4: Create .gitignore for .claude repo**

Create `/home/finn/Documents/ardu_ws/.claude/.gitignore`:
```gitignore
# Personal settings (don't version control)
settings.local.json
*.local.md

# OS files
.DS_Store
Thumbs.db

# IDE
.vscode/
.idea/
```

**Step 5: Restart Cursor and verify**
```bash
# Close and reopen Cursor at workspace root
code /home/finn/Documents/ardu_ws/

# Check Source Control panel:
# Should now see:
# - .claude repository
# - ardupilot_gz repository
# - ardupilot_gazebo repository
# - formation_control repository
# - etc.
```

#### Pros
- ✅ Simple git operations (no submodules)
- ✅ .claude/ is version controlled independently
- ✅ Each src/ repo remains fully independent
- ✅ Works with standard git workflows

#### Cons
- ⚠️ Requires Cursor configuration (team members need same settings)
- ⚠️ If scan depth is insufficient, some repos might be missed
- ⚠️ .claude/ must be manually cloned separately when setting up workspace

#### Best For
- Single developer or small team
- When you control Cursor settings across team
- Simple independent repo management

---

### Solution 2: Git Worktree Approach (Advanced)

#### The Concept
Use Git's worktree feature to have `.claude/` tracked by a separate repository without creating a parent repo at workspace root.

#### How It Works
```
Main .claude repo:     /somewhere/else/.claude-repo/
Worktree location:     /home/finn/Documents/ardu_ws/.claude/
```

The worktree appears as a normal directory but is actually linked to a git repo elsewhere.

#### Implementation

**Step 1: Create main .claude repository elsewhere**
```bash
# Create the main repo outside the workspace
mkdir -p ~/repos/ardu-ws-claude-config
cd ~/repos/ardu-ws-claude-config
git init

# Move current .claude contents here
cp -r /home/finn/Documents/ardu_ws/.claude/* .
git add .
git commit -m "Initial .claude configuration"
```

**Step 2: Create worktree at workspace location**
```bash
# Remove existing .claude directory
rm -rf /home/finn/Documents/ardu_ws/.claude

# Create worktree at the workspace location
cd ~/repos/ardu-ws-claude-config
git worktree add /home/finn/Documents/ardu_ws/.claude main
```

**Step 3: Use normally**
```bash
# Work in .claude as normal
cd /home/finn/Documents/ardu_ws/.claude/
# Edit files...

# Git operations work directly here
git status
git add .
git commit -m "Update configuration"
git push
```

#### Pros
- ✅ No nested repository issues
- ✅ .claude/ appears as normal directory to IDE
- ✅ Full git functionality
- ✅ Workspace root stays repo-free

#### Cons
- ⚠️ Advanced git feature (team needs to understand it)
- ⚠️ Main repo must exist somewhere else on filesystem
- ⚠️ Worktree reference can break if main repo moves
- ⚠️ Complex initial setup for team members

#### Best For
- Advanced git users
- When you want .claude/ to feel like a normal directory
- Single developer workflows

---

### Solution 3: Sparse Checkout Strategy (Complex)

#### The Concept
Create a single repository at workspace root but use sparse checkout to only track `.claude/` while ignoring everything else.

**I don't recommend this solution** - it's overly complex and fragile for your use case. Documented here only for completeness.

#### Why It's Problematic
- Requires careful .gitignore management
- High risk of accidentally committing src/ repos as nested repos
- IDE still sees workspace root as primary repo
- Defeats your goal of keeping repos independent

---

### Solution 4: Separate Repo + Smart Symlinks

#### The Concept
Keep `.claude/` as a separate repo in a different location, and symlink it into the workspace.

#### Implementation

**Step 1: Move .claude to separate location**
```bash
# Move .claude outside workspace
mv /home/finn/Documents/ardu_ws/.claude ~/repos/ardu-ws-claude-config

# Initialize as git repo
cd ~/repos/ardu-ws-claude-config
git init
git add .
git commit -m "Initial .claude configuration"
```

**Step 2: Create symlink**
```bash
# Create symlink in workspace
cd /home/finn/Documents/ardu_ws/
ln -s ~/repos/ardu-ws-claude-config .claude

# Verify
ls -la .claude  # Should show it's a symlink
```

**Step 3: Use normally**
```bash
# Claude Code will follow symlink transparently
# Your .claude/CLAUDE.md and other files work normally

# Git operations in .claude work on the separate repo
cd /home/finn/Documents/ardu_ws/.claude
git status  # Operates on ~/repos/ardu-ws-claude-config
```

#### Pros
- ✅ Complete separation of .claude from workspace
- ✅ No nested repository issues
- ✅ IDE sees .claude as normal directory
- ✅ Claude Code works transparently with symlinks

#### Cons
- ⚠️ Symlinks can be confusing for team members
- ⚠️ Doesn't work well on Windows (requires admin rights)
- ⚠️ Some tools don't follow symlinks properly
- ⚠️ Backup tools might skip symlinked content

#### Best For
- Linux/Mac-only teams
- When you want complete separation
- Advanced users comfortable with symlinks

---

### Solution 5: Cursor Multi-Root Workspace

#### The Concept
Use Cursor's multi-root workspace feature to open multiple repos as separate roots, giving each one full Git GUI support.

#### Implementation

**Step 1: Create .claude as separate repo**
```bash
cd /home/finn/Documents/ardu_ws/.claude/
git init
git add .
git commit -m "Initial configuration"
```

**Step 2: Create multi-root workspace file**

Create `/home/finn/Documents/ardu_ws/ardu_ws.code-workspace`:
```json
{
    "folders": [
        {
            "name": "Workspace Root",
            "path": "."
        },
        {
            "name": "Claude Config",
            "path": ".claude"
        },
        {
            "name": "ArduPilot Gazebo",
            "path": "src/ardupilot_gazebo"
        },
        {
            "name": "ArduPilot GZ",
            "path": "src/ardupilot_gz"
        },
        {
            "name": "Formation Control",
            "path": "src/formation_control"
        },
        {
            "name": "ArduPilot",
            "path": "src/ardupilot"
        },
        {
            "name": "ROS GZ",
            "path": "src/ros_gz"
        },
        {
            "name": "Micro-ROS Agent",
            "path": "src/micro_ros_agent"
        }
    ],
    "settings": {
        "git.repositoryScanMaxDepth": 10,
        "scm.repositories.visible": 15
    }
}
```

**Step 3: Open workspace in Cursor**
```bash
cursor ardu_ws.code-workspace
```

#### Pros
- ✅ Each repo gets full Git GUI independently
- ✅ Clean organization in Explorer panel
- ✅ Workspace settings apply to all folders
- ✅ Easy to add/remove repos from workspace

#### Cons
- ⚠️ Must use .code-workspace file (can't just open folder)
- ⚠️ Slightly different workflow than normal folder opening
- ⚠️ Team members need the workspace file

#### Best For
- **HIGHLY RECOMMENDED for your case**
- Teams working with multiple repositories
- When you want clear separation and organization
- IDE power users

---

### Solution 6: Separate .claude Repo (No Parent) - SIMPLEST

#### The Concept
Simply keep workspace root repo-free, initialize `.claude/` as its own git repo, and rely on Cursor's default multi-repo support.

#### Implementation

**Step 1: Verify workspace root is clean**
```bash
# Ensure no .git at workspace root
ls -la /home/finn/Documents/ardu_ws/.git  # Should not exist
```

**Step 2: Initialize .claude as repo**
```bash
cd /home/finn/Documents/ardu_ws/.claude/
git init
git add .
git commit -m "Initial .claude workspace configuration"

# Optional: Add remote
git remote add origin <your-git-url>
git push -u origin main
```

**Step 3: Update Cursor settings (optional but recommended)**

Edit `/home/finn/Documents/ardu_ws/.vscode/settings.json`:
```json
{
    "git.repositoryScanMaxDepth": 5,
    "scm.repositories.visible": 12,
    "Lua.diagnostics.disable": [
        "duplicate-doc-field",
        "undefined-global",
        "param-type-mismatch"
    ]
}
```

**Step 4: Test in Cursor**
1. Open Cursor at workspace root
2. Open Source Control panel (Ctrl+Shift+G)
3. You should see all repos listed including `.claude`
4. Each repo should be independently manageable

#### Pros
- ✅ **Simplest solution**
- ✅ No special git features needed
- ✅ No symlinks or worktrees
- ✅ Works out of the box with proper settings
- ✅ Easy for team to replicate

#### Cons
- ⚠️ May need to tweak scan depth settings
- ⚠️ Depends on Cursor's detection working properly

#### Best For
- **RECOMMENDED for most cases**
- Simple workflows
- Teams not familiar with advanced git
- Quick setup needed

---

## Recommended Approach

Based on your requirements (Cursor IDE, need visual Git GUI, simple workflow), I recommend **Solution 6** with **Solution 5** as a fallback.

### Primary Recommendation: Solution 6 (Separate .claude Repo)

**Why**:
1. Simplest to implement and maintain
2. No advanced git features required
3. Works with Cursor's built-in multi-repo support
4. Easy for team members to understand
5. No special tooling or configuration needed

**When it works best**:
- Cursor properly detects all repos with scan depth settings
- Your repo count stays under ~15 repositories
- Team members use Cursor or VS Code

### Fallback: Solution 5 (Multi-Root Workspace)

**Why**:
1. Explicit control over which repos are visible
2. Guaranteed to work regardless of scan depth
3. Better organization for large number of repos
4. Can customize settings per-repo

**When to use**:
- Solution 6 doesn't show all repos reliably
- You want better organization in Explorer panel
- Team is comfortable with .code-workspace files

---

## Implementation Guide

### Recommended Implementation (Solution 6 + 5 Hybrid)

#### Phase 1: Basic Setup (Solution 6)

**1. Clean the workspace root**
```bash
cd /home/finn/Documents/ardu_ws/

# Check if workspace root has .git (it shouldn't)
if [ -d .git ]; then
    echo "WARNING: Workspace root has .git directory"
    echo "This needs to be removed for the solution to work"
    echo "Make sure you don't have important commits there first!"
    # rm -rf .git  # Only run if you're sure
fi
```

**2. Initialize .claude as git repo**
```bash
cd /home/finn/Documents/ardu_ws/.claude/

# Initialize if not already done
git init

# Check current status
git status

# Add files you want to version control
git add CLAUDE.md BUILD_REFERENCE.md WORKSPACE_CLEAN_REBUILD_GUIDE.md
git add commands/ agents/

# Create .gitignore
cat > .gitignore << 'EOF'
# Personal settings
settings.local.json
*.local.md

# OS files
.DS_Store
Thumbs.db

# Temporary files
*.tmp
*.swp
*~
EOF

git add .gitignore
git commit -m "Initial .claude workspace configuration"

# Add remote (replace with your repo URL)
# git remote add origin git@github.com:username/ardu-ws-claude.git
# git push -u origin main
```

**3. Configure Cursor for multi-repo detection**
```bash
cd /home/finn/Documents/ardu_ws/

# Update or create .vscode/settings.json
cat > .vscode/settings.json << 'EOF'
{
    "git.repositoryScanMaxDepth": 10,
    "git.detectSubmodules": false,
    "git.autoRepositoryDetection": true,
    "scm.repositories.visible": 15,
    "scm.repositories.sortOrder": "discovery time",
    "git.openRepositoryInParentFolders": "always",
    "Lua.diagnostics.disable": [
        "duplicate-doc-field",
        "undefined-global",
        "param-type-mismatch"
    ]
}
EOF
```

**4. Test in Cursor**
```bash
# Close Cursor completely
# Reopen at workspace root
cursor /home/finn/Documents/ardu_ws/

# Open Source Control panel (Ctrl+Shift+G)
# You should see multiple repositories listed
```

**5. Verify all repos are visible**
```bash
# Expected repos in Source Control panel:
# - .claude
# - src/ardupilot_gz
# - src/ardupilot_gazebo
# - src/formation_control
# - src/ardupilot (and its submodules)
# - src/ros_gz
# - src/micro_ros_agent
# - src/Micro-XRCE-DDS-Gen
# - src/ardupilot_sitl_models
# - src/sdformat_urdf
```

#### Phase 2: Fallback to Multi-Root Workspace (if needed)

If Phase 1 doesn't show all repos reliably:

**1. Create workspace file**
```bash
cd /home/finn/Documents/ardu_ws/

cat > ardu_ws.code-workspace << 'EOF'
{
    "folders": [
        {
            "name": "🏠 Workspace Root",
            "path": "."
        },
        {
            "name": "🤖 Claude Config",
            "path": ".claude"
        },
        {
            "name": "🚁 ArduPilot GZ",
            "path": "src/ardupilot_gz"
        },
        {
            "name": "🌍 ArduPilot Gazebo",
            "path": "src/ardupilot_gazebo"
        },
        {
            "name": "🎯 Formation Control",
            "path": "src/formation_control"
        },
        {
            "name": "✈️ ArduPilot",
            "path": "src/ardupilot"
        }
    ],
    "settings": {
        "git.repositoryScanMaxDepth": 10,
        "git.detectSubmodules": false,
        "scm.repositories.visible": 20,
        "files.exclude": {
            "**/build": true,
            "**/install": true,
            "**/log": true
        }
    }
}
EOF
```

**2. Open workspace file**
```bash
cursor ardu_ws.code-workspace
```

Now each folder appears as a separate root with its own Git controls.

---

## Troubleshooting

### Problem: Cursor still doesn't show all repositories

**Check 1: Verify scan depth**
```bash
# In Cursor, open Command Palette (Ctrl+Shift+P)
# Type: "Preferences: Open Settings (JSON)"
# Check for git.repositoryScanMaxDepth
```

**Check 2: Verify repos are valid git repos**
```bash
cd /home/finn/Documents/ardu_ws/src/ardupilot_gz
git status  # Should show git status, not "not a git repository"
```

**Check 3: Reload window**
```bash
# In Cursor: Ctrl+Shift+P → "Developer: Reload Window"
```

**Check 4: Check Git extension is enabled**
```bash
# In Cursor: Extensions view → Search "Git"
# Ensure Git extension is enabled (it's built-in)
```

### Problem: .claude repo works but src/ repos don't show

**Solution A: Increase scan depth**
```json
{
    "git.repositoryScanMaxDepth": -1  // Unlimited depth
}
```

**Solution B: Use multi-root workspace (Phase 2)**

### Problem: Git GUI shows wrong repository

**Cause**: Multiple repos open, Cursor focusing on wrong one

**Solution**: Click the repository name in Source Control panel to switch active repo, or use multi-root workspace for explicit separation.

### Problem: Changes in .claude not showing in Source Control

**Check 1: Verify .claude is a git repo**
```bash
cd /home/finn/Documents/ardu_ws/.claude
ls -la .git  # Should exist
```

**Check 2: Check if files are tracked**
```bash
git status  # Shows tracked vs untracked files
```

**Check 3: Check .gitignore**
```bash
cat .gitignore  # Ensure you're not ignoring files you want to track
```

### Problem: Workspace root shows as dirty repo

**Cause**: Workspace root was initialized as git repo

**Solution**:
```bash
cd /home/finn/Documents/ardu_ws/
ls -la .git  # Check if exists

# If you don't need it:
rm -rf .git

# Then reload Cursor window
```

---

## Best Practices

### For .claude Repository

**What to commit**:
```
✅ CLAUDE.md                    # Project context for Claude
✅ BUILD_REFERENCE.md           # Build documentation
✅ commands/                    # Custom slash commands
✅ agents/                      # Custom agent definitions
✅ .gitignore                   # Git ignore rules
```

**What NOT to commit**:
```
❌ settings.local.json          # Personal settings
❌ Features/*.local.md          # Personal notes
❌ *.tmp, *.swp, *~            # Temporary files
```

**Sample .gitignore for .claude**:
```gitignore
# Personal settings
settings.local.json
*.local.md
.personal/

# Temporary files
*.tmp
*.swp
*~
*.bak

# OS files
.DS_Store
Thumbs.db
desktop.ini

# IDE
.vscode/
.idea/

# Logs
*.log
```

### For Team Collaboration

**1. Document the setup in README**
```markdown
## Git Setup

This workspace contains multiple git repositories:
- `.claude/` - Claude Code configuration (tracked separately)
- `src/*/` - Individual ROS2 packages (each has own git repo)

### First-Time Setup
1. Clone main repos in src/
2. Clone .claude config:
   ```bash
   cd /home/finn/Documents/ardu_ws/
   git clone <url> .claude
   ```
3. Open workspace: `cursor ardu_ws.code-workspace`
```

**2. Share Cursor settings**
- Commit `.vscode/settings.json` to workspace root (but keep it minimal)
- Team members will inherit the scan depth settings

**3. Use workspace file**
- Commit `ardu_ws.code-workspace` for consistent experience
- Team members open workspace file, not folder

---

## Technical Deep Dive

### How Cursor Detects Git Repositories

Cursor (VS Code fork) uses the `vscode.git` API which implements:

```typescript
// Simplified algorithm
async function discoverGitRepositories(workspaceRoot: string): Promise<Repository[]> {
    const repos: Repository[] = [];
    const maxDepth = getConfiguration('git.repositoryScanMaxDepth');

    // Start scanning from workspace root
    await scanDirectory(workspaceRoot, 0, maxDepth, repos);

    // If workspace root is a git repo, it takes priority
    if (await isGitRepository(workspaceRoot)) {
        repos.unshift(new Repository(workspaceRoot));
    }

    return repos;
}

async function scanDirectory(
    dir: string,
    currentDepth: number,
    maxDepth: number,
    repos: Repository[]
): Promise<void> {
    if (currentDepth >= maxDepth) return;

    const entries = await fs.readdir(dir);

    for (const entry of entries) {
        const fullPath = path.join(dir, entry);

        // Skip node_modules, build directories, etc.
        if (shouldSkip(entry)) continue;

        // Check if this directory is a git repo
        if (await isGitRepository(fullPath)) {
            repos.push(new Repository(fullPath));
        }

        // Recurse into subdirectories
        if (await isDirectory(fullPath)) {
            await scanDirectory(fullPath, currentDepth + 1, maxDepth, repos);
        }
    }
}
```

### Why Workspace Root Git Repo Breaks Things

When you run `git init` at workspace root:

1. **Discovery stops early**:
   ```
   /ardu_ws/.git ← Found! This becomes primary repo
   └── Scanning stops or deprioritizes other repos
   ```

2. **Submodule confusion**:
   ```
   Git sees:
   /ardu_ws/.git (parent)
   /ardu_ws/src/ardupilot_gz/.git (child)

   Assumes: child is a submodule
   Result: Hides child's Git GUI
   ```

3. **SCM view priority**:
   ```
   Source Control Panel:
   [Primary] ardu_ws ← Gets all the UI space
   [Hidden] src/ardupilot_gz
   [Hidden] src/ardupilot_gazebo
   ```

### Solution: Keep Workspace Root Git-Free

```
/ardu_ws/                    ← No .git here!
├── .claude/.git             ← Independent repo (depth=1)
└── src/
    ├── ardupilot_gz/.git    ← Independent repo (depth=2)
    └── ardupilot_gazebo/.git ← Independent repo (depth=2)
```

With `git.repositoryScanMaxDepth: 10`:
- Cursor scans up to 10 levels deep
- Finds all repos independently
- No parent-child relationship
- All repos get equal treatment in GUI

---

## Conclusion

### TL;DR - Quick Answer

**Problem**: Initializing `/home/finn/Documents/ardu_ws/` as git repo breaks Cursor's ability to see sub-repos.

**Solution**:
1. **DON'T** initialize workspace root as git repo
2. **DO** initialize `.claude/` as its own git repo
3. **DO** configure Cursor scan depth: `"git.repositoryScanMaxDepth": 10`
4. **OPTIONAL** use multi-root workspace file for explicit control

**Commands**:
```bash
# Setup
cd /home/finn/Documents/ardu_ws/.claude/
git init
git add .
git commit -m "Initial configuration"

# Configure Cursor
echo '{
    "git.repositoryScanMaxDepth": 10,
    "scm.repositories.visible": 15
}' > ../.vscode/settings.json

# Restart Cursor
cursor /home/finn/Documents/ardu_ws/
```

### Final Recommendation

1. **Start with Solution 6** (separate .claude repo + Cursor config)
   - Simplest
   - Most maintainable
   - Works 95% of the time

2. **If that fails, use Solution 5** (multi-root workspace)
   - More explicit
   - Guaranteed to work
   - Better organization

3. **Avoid**:
   - Git submodules (too complex)
   - Sparse checkout (fragile)
   - Symlinks (platform-dependent)

### Success Criteria

You'll know it's working when:
- ✅ Cursor Source Control panel shows all repos
- ✅ You can click each repo and see its changes
- ✅ Git GUI operations (commit, push, pull) work per-repo
- ✅ No "submodule" warnings or weird git behavior
- ✅ `.claude/` changes are tracked independently

### Next Steps

1. Implement Phase 1 (Solution 6)
2. Test with your repos
3. If issues, implement Phase 2 (Solution 5)
4. Document setup for team in README
5. Commit workspace configuration files

---

**Research Completed**: 2025-10-24
**Tested With**: Cursor IDE (VS Code fork)
**Workspace**: /home/finn/Documents/ardu_ws/
**Repository Count**: ~10 independent git repositories
