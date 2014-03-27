echo ""Usage: ./git.sh "commit message"

git add -A;
git commit -m "$1"; #"name fast to async";
git push origin master;
