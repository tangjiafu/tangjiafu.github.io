#初始化一本书的文件结构
section=4
for a in $(seq 1 ${section})
do
#文件夹不存在创建
if [ ! -d chapter0${a} ];then
  mkdir chapter0${a}
  cd chapter0${a}
  gitbook init
  cd ..
fi
done
