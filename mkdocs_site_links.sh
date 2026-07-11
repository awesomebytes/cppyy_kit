#!/bin/bash
# Regenerate the mkdocs_site/ symlink mirror (M4 docs site).
#
# WHY symlinks that MIRROR the repo layout: the read-only kit/docs .md files use
# relative links assuming the repo layout (kit dirs are siblings of docs/, e.g.
# cv_kit/SKILL.md -> ../docs/tutorials/..., bt_kit/WHY.md -> REPORT.md). Rendering
# them from a docs_dir that mirrors that layout makes those links resolve without
# editing the originals. We symlink the .md files we put in the nav (not whole kit
# dirs) to avoid pulling *.py / demos / tests as static assets.
set -euo pipefail
cd "$(dirname "$0")"
SITE=mkdocs_site
# Wipe only the generated symlinks/dirs, keep the authored pages (index.md, etc.).
rm -rf "$SITE/docs" "$SITE/PLAN.md"
for k in cppyy_kit rclcpp_kit bt_kit pcl_kit ompl_kit nav2_kit moveit_kit control_kit cv_kit dbow_kit; do
  rm -rf "$SITE/$k"
done

# Shared docs dir (COMMON_PATTERNS / FREEZE / ARCHITECTURE_V2 / tutorials/) + PLAN.
ln -s ../docs "$SITE/docs"
ln -s ../PLAN.md "$SITE/PLAN.md"

# Per-kit trio (+ extras), mirrored under <kit>/ so intra-kit links resolve.
for k in rclcpp_kit bt_kit pcl_kit ompl_kit nav2_kit moveit_kit control_kit cv_kit dbow_kit; do
  mkdir -p "$SITE/$k"
  for f in "$k"/*.md; do
    [ -e "$f" ] || continue
    b="$(basename "$f")"
    ln -s "../../$k/$b" "$SITE/$k/$b"
  done
done
echo "mkdocs_site symlink mirror regenerated."

# Accelerate skill (M5)
mkdir -p mkdocs_site/skills/cppyy-accelerate
ln -sfn ../../../skills/cppyy-accelerate/SKILL.md mkdocs_site/skills/cppyy-accelerate/SKILL.md
ln -sfn ../../../skills/cppyy-accelerate/WALKTHROUGH.md mkdocs_site/skills/cppyy-accelerate/WALKTHROUGH.md
