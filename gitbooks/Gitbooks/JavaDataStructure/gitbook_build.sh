
if [ -d "./book" ];then
rm -rf ./book
fi
if [ -d "./_book" ];then
rm -rf ./_book
fi
gitbook install
gitbook build
mv _book book

