# How we work together (Git)

So we don’t overwrite or break each other’s work.

---

## Rules

1. **Pull before you start** — Get latest `main` (or your branch) before editing.
2. **Use a branch for your work** — Don’t commit directly to `main`.
3. **Build and test before you push** — `catkin_make` and run what you changed.
4. **Say what you’re changing** — Especially if someone else might be in the same file.

---

## Simple workflow

**Start working**

```bash
git pull origin main
git checkout -b feature/short-name   # e.g. feature/vision-blocks
```

**While working**

```bash
# Build and test
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

**When you’re done**

```bash
git add <files you changed>
git commit -m "Add: what you did"
git push origin feature/short-name
```

Then open a pull request (or tell the team to pull your branch). Don’t push to `main` yourself.

---

## If you get a merge conflict

1. `git status` — see which files conflict.
2. Open the file: you’ll see `<<<<<<<`, `=======`, `>>>>>>>`. Edit so the file is correct (keep your part, their part, or both).
3. `git add <file>` then `git commit -m "Fix: merge conflict in <file>"`.

---

## Don’t

- Push to `main` (use a branch and merge via PR or team lead).
- Force-push (`git push --force`) on shared branches.
- Commit without building: `catkin_make` must pass.
- Commit build outputs (e.g. `build/`, `devel/`) — they’re in `.gitignore`.

---

## Who does what (to avoid conflicts)

Current assignments:

- **Vision** (RGB + depth → blocks, player selection): **Ali**  
- **Game logic** (sequence, score, states): **Sinan**  
- **Motion** (Panda point + home): **Izat**  
- **Launch files, config, docs**: **Boburjon**  

If you need to change someone else’s part, just tell them so you don’t conflict.
