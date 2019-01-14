# FRCMinotaur2019
Private repo for 2019 Team 1369 code.
### DELETE THE FOLDER FOR THE OLD REPO BEFORE CLONING THIS ONE
To open this project in VS Code click File/Open Workspace and select FRCMinotaur2019.code-workspace

## Setup
To use with git bash, choose a folder to clone into, then run:
```
git clone https://github.com/FRCMinotaur/FRCMinotaur2019.git
```
## Push code to Github
```
git add src
git commit -m "your commit message"
git push origin your-branch-name
```
If you are on the master branch, your-branch-name is just master
## Branching
Create a branch anytime multiple people are implementing different code at the same time.
Branch names should be descriptive and all lowercase (No spaces)
Create new branch with:
```
git checkout -b new-branch-name
```
Switch to a different branch at anytime with:
```
git checkout your-branch-name
```
## Merging
After your feature is done being implemented on a given branch you should merge it back into master:
```
git checkout master
git merge your-branch-name
```
Contact me if you get merge conflicts and don't know how to fix them.

If you want to delete the branch after doing this run:
```
git branch -d your-branch-name
```
Then commit and push your changes.
## Replace Local Changes
If you have changes on your computer that you do not care about and just want to have the code from Github, run:
```
git fetch origin
git reset --hard origin/your-branch-name
```
## Tagging Releases
At big points during the season (End of build season, before and after competitions, or the addition of major features, etc.) you should tag releases so you can easily refer back to that version of the code if something breaks. You should only tag releases on the master branch (AKA after code from all other branches has been merged into master) Run:
```
git checkout master
git add src
git commit -m "your commit message"
git tag -a v1.4 -m "your tag message"
git push origin master
```
Replace v1.4 with whatever version number you choose.  I would suggest tagging the code at the end of build season as v1.0 and after that increment the least significant number for a small change and the most significant number for a large change.

## Good Luck with the Rest of the 2019 Season!


