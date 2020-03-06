### Chpater08. Redis详解

### 0. 引用

- <https://blog.csdn.net/qq_33423418/article/details/101351944>
- <https://www.runoob.com/redis/redis-keys.html>

### 1. 简介

REmote DIctionary Server(Redis) 是一个由Salvatore Sanfilippo写的key-value存储系统。

Redis是一个开源的使用ANSI C语言编写、遵守BSD协议、支持网络、可基于内存亦可持久化的日志型、Key-Value数据库，并提供多种语言的API。

它通常被称为数据结构服务器，因为值（value）可以是 字符串(String), 哈希(Hash), 列表(list), 集合(sets) 和 有序集合(sorted sets)等类型。

Redis 与其他 key - value 缓存产品有以下三个特点：

- Redis支持数据的持久化，可以将内存中的数据保存在磁盘中，重启的时候可以再次加载进行使用。
- Redis不仅仅支持简单的key-value类型的数据，同时还提供list，set，zset，hash等数据结构的存储。
- Redis支持数据的备份，即master-slave模式的数据备份。

优越性

- 性能极高 – Redis能读的速度是110000次/s,写的速度是81000次/s 。
- 丰富的数据类型 – Redis支持二进制案例的 Strings, Lists, Hashes, Sets 及 Ordered Sets 数据类型操作。
- 原子 – Redis的所有操作都是原子性的，意思就是要么成功执行要么失败完全不执行。单个操作是原子性的。多个操作也支持事务，即原子性，通过MULTI和EXEC指令包起来。
- 丰富的特性 – Redis还支持 publish/subscribe, 通知, key 过期等等特性。

### 2. 安装与配置

```shell
sudo apt-get install redis-server
```

启动

``` shell
redis-server
```

客户端

```shell
reids-cli
```

Redis 的配置文件位于 Redis 安装目录下，文件名为 **redis.conf**,通过命令CONFIG GET获取配置信息，CONFIG SET设置信息。

#### 配置项说明

| 序号 | 配置项                                                       | 说明                                                         |
| :--- | :----------------------------------------------------------- | :----------------------------------------------------------- |
| 1    | `daemonize no`                                               | Redis 默认不是以守护进程的方式运行，可以通过该配置项修改，使用 yes 启用守护进程（Windows 不支持守护线程的配置为 no ） |
| 2    | `pidfile /var/run/redis.pid`                                 | 当 Redis 以守护进程方式运行时，Redis 默认会把 pid 写入 /var/run/redis.pid 文件，可以通过 pidfile 指定 |
| 3    | `port 6379`                                                  | 指定 Redis 监听端口，默认端口为 6379，作者在自己的一篇博文中解释了为什么选用 6379 作为默认端口，因为 6379 在手机按键上 MERZ 对应的号码，而 MERZ 取自意大利歌女 Alessia Merz 的名字 |
| 4    | `bind 127.0.0.1`                                             | 绑定的主机地址                                               |
| 5    | `timeout 300`                                                | 当客户端闲置多长时间后关闭连接，如果指定为 0，表示关闭该功能 |
| 6    | `loglevel notice`                                            | 指定日志记录级别，Redis 总共支持四个级别：debug、verbose、notice、warning，默认为 notice |
| 7    | `logfile stdout`                                             | 日志记录方式，默认为标准输出，如果配置 Redis 为守护进程方式运行，而这里又配置为日志记录方式为标准输出，则日志将会发送给 /dev/null |
| 8    | `databases 16`                                               | 设置数据库的数量，默认数据库为0，可以使用SELECT 命令在连接上指定数据库id |
| 9    | `save <seconds> <changes>`<br />Redis 默认配置文件中提供了三个条件：<br />save 900 1<br />save 300 10**<br />**save 60 10000<br /> | 指定在多长时间内，有多少次更新操作，就将数据同步到数据文件，可以多个条件配合。<br />redis默认配置有三个条件，满足一个即进行持久化，分别表示 900 秒（15 分钟）内有 1 个更改，300 秒（5 分钟）内有 10 个更改以及 60 秒内有 10000 个更改。 |
| 10   | `rdbcompression yes`                                         | 指定存储至本地数据库时是否压缩数据，默认为 yes，Redis 采用 LZF 压缩，如果为了节省 CPU 时间，可以关闭该选项，但会导致数据库文件变的巨大 |
| 11   | `dbfilename dump.rdb`                                        | 指定本地数据库文件名，默认值为 dump.rdb                      |
| 12   | `dir ./`                                                     | 指定本地数据库存放目录                                       |
| 13   | `slaveof <masterip> <masterport>`                            | 设置当本机为 slav 服务时，设置 master 服务的 IP 地址及端口，在 Redis 启动时，它会自动从 master 进行数据同步 |
| 14   | `masterauth <master-password>`                               | 当 master 服务设置了密码保护时，slav 服务连接 master 的密码  |
| 15   | `requirepass foobared`                                       | 设置 Redis 连接密码，如果配置了连接密码，客户端在连接 Redis 时需要通过 AUTH <password> 命令提供密码，默认关闭 |
| 16   | ` maxclients 128`                                            | 设置同一时间最大客户端连接数，默认无限制，Redis 可以同时打开的客户端连接数为 Redis 进程可以打开的最大文件描述符数，如果设置 maxclients 0，表示不作限制。当客户端连接数到达限制时，Redis 会关闭新的连接并向客户端返回 max number of clients reached 错误信息 |
| 17   | `maxmemory <bytes>`                                          | 指定 Redis 最大内存限制，Redis 在启动时会把数据加载到内存中，达到最大内存后，Redis 会先尝试清除已到期或即将到期的 Key，当此方法处理 后，仍然到达最大内存设置，将无法再进行写入操作，但仍然可以进行读取操作。Redis 新的 vm 机制，会把 Key 存放内存，Value 会存放在 swap 区 |
| 18   | `appendonly no`                                              | 指定是否在每次更新操作后进行日志记录，Redis 在默认情况下是异步的把数据写入磁盘，如果不开启，可能会在断电时导致一段时间内的数据丢失。因为 redis 本身同步数据文件是按上面 save 条件来同步的，所以有的数据会在一段时间内只存在于内存中。默认为 no |
| 19   | `appendfilename appendonly.aof`                              | 指定更新日志文件名，默认为 appendonly.aof                    |
| 20   | `appendfsync everysec`                                       | 指定更新日志条件，共有 3 个可选值：**no**：表示等操作系统进行数据缓存同步到磁盘（快）**always**：表示每次更新操作后手动调用 fsync() 将数据写到磁盘（慢，安全）**everysec**：表示每秒同步一次（折中，默认值） |
| 21   | `vm-enabled no`                                              | 指定是否启用虚拟内存机制，默认值为 no，简单的介绍一下，VM 机制将数据分页存放，由 Redis 将访问量较少的页即冷数据 swap 到磁盘上，访问多的页面由磁盘自动换出到内存中（在后面的文章我会仔细分析 Redis 的 VM 机制） |
| 22   | `vm-swap-file /tmp/redis.swap`                               | 虚拟内存文件路径，默认值为 /tmp/redis.swap，不可多个 Redis 实例共享 |
| 23   | `vm-max-memory 0`                                            | 将所有大于 vm-max-memory 的数据存入虚拟内存，无论 vm-max-memory 设置多小，所有索引数据都是内存存储的(Redis 的索引数据 就是 keys)，也就是说，当 vm-max-memory 设置为 0 的时候，其实是所有 value 都存在于磁盘。默认值为 0 |
| 24   | `vm-page-size 32`                                            | Redis swap 文件分成了很多的 page，一个对象可以保存在多个 page 上面，但一个 page 上不能被多个对象共享，vm-page-size 是要根据存储的 数据大小来设定的，作者建议如果存储很多小对象，page 大小最好设置为 32 或者 64bytes；如果存储很大大对象，则可以使用更大的 page，如果不确定，就使用默认值 |
| 25   | `vm-pages 134217728`                                         | 设置 swap 文件中的 page 数量，由于页表（一种表示页面空闲或使用的 bitmap）是在放在内存中的，，在磁盘上每 8 个 pages 将消耗 1byte 的内存。 |
| 26   | `vm-max-threads 4`                                           | 设置访问swap文件的线程数,最好不要超过机器的核数,如果设置为0,那么所有对swap文件的操作都是串行的，可能会造成比较长时间的延迟。默认值为4 |
| 27   | `glueoutputbuf yes`                                          | 设置在向客户端应答时，是否把较小的包合并为一个包发送，默认为开启 |
| 28   | `hash-max-zipmap-entries 64 hash-max-zipmap-value 512`       | 指定在超过一定的数量或者最大的元素超过某一临界值时，采用一种特殊的哈希算法 |
| 29   | `activerehashing yes`                                        | 指定是否激活重置哈希，默认为开启（后面在介绍 Redis 的哈希算法时具体介绍） |
| 30   | `include /path/to/local.conf`                                | 指定包含其它的配置文件，可以在同一主机上多个Redis实例之间使用同一份配置文件，而同时各个实例又拥有自己的特定配置文件 |

### 3. 数据类型

redis数据类型包括字符串（String），哈希（Hash），列表（List），Set（集合），zset(sorted set：有序集合)

**String.**

- string 是 redis 最基本的类型，一个 key 对应一个 value。

- string 类型是二进制安全的。即 redis 的 string 可以包含任何数据。比如jpg图片或者序列化的对象。

- string 类型是 Redis 最基本的数据类型，string 类型的值最大能存储 512MB。

**Hash.**

- Redis hash 是一个键值(key=>value)对集合。

- Redis hash 是一个 string 类型的 field 和 value 的映射表，hash 特别适合用于存储对象。
- 每个 hash 可以存储 232 -1 键值对（40多亿)

**List**

- Redis 列表是简单的字符串列表，按照插入顺序排序。你可以添加一个元素到列表的头部（左边）或者尾部（右边）。

- 列表最多可存储 232 - 1 元素 (4294967295, 每个列表可存储40多亿)。

**Set**

- Redis 的 Set 是 string 类型的无序集合。

- 集合是通过哈希表实现的，所以添加，删除，查找的复杂度都是 O(1)。
- 集合中最大的成员数为 232 - 1(4294967295, 每个集合可存储40多亿个成员)。

zset

Redis zset 和 set 一样也是string类型元素的集合,且不允许重复的成员。

不同的是每个元素都会关联一个double的权重。redis通过权重来为集合中的成员进行从小到大的排序。

zset的成员是唯一的,但分数(score)却可以重复。

### 4. 命令

Redis 命令用于在 redis 服务上执行操作。

要在 redis 服务上执行命令需要一个 redis 客户端。Redis 客户端在我们之前下载的的 redis 的安装包中。

Redis命令不区分大小写。

```shell
redis-cli
```

如果需要在远程 redis 服务上执行命令，同样我们使用的也是 **redis-cli** 命令。

```shell
redis-cli -h host -p port -a password
```

#### **通用命令**

> - DEL key:删除key
> - DUMP key：序列化给定key，返回被序列化的值
>
> - EXISTS key：检查key是否存在
> - EXPIRE key second：为key设定过期时间
> - TTL key：返回key剩余时间
> - PERSIST key：移除key的过期时间，key将持久保存
> - KEY pattern：查询所有符号给定模式的key
> - RANDOM key：随机返回一个key
> - RANAME key newkey：修改key的名称
> - MOVE key db：移动key至指定数据库中
> - TYPE key：返回key所储存的值的类型
>

EXPIRE key second的使用场景：

1. 限时的优惠活动
2. 网站数据缓存
3. 手机验证码
4. 限制网站访客频率

key的命名建议

1. key不要太长，尽量不要超过1024字节。不仅消耗内存，也会降低查找的效率
2. key不要太短，太短可读性会降低
3. 在一个项目中，key最好使用统一的命名模式，如user:123:password
4. key区分大小写

#### string命令

> - setkey_name value：命令不区分大小写，但是key_name区分大小写
> - SETNX key value：当key不存在时设置key的值。（SET if Not eXists）
> - get key_name
> - GETRANGE key start end：获取key中字符串的子字符串，从start开始，end结束
> - MGET key1 [key2 …]：获取多个key
> - GETSET KEY_NAME VALUE：设定key的值，并返回key的旧值。当key不存在，返回nil
> - STRLEN key：返回key所存储的字符串的长度
> - INCR KEY_NAME ：INCR命令key中存储的值+1,如果不存在key，则key中的值话先被初始化为0再加1
> - INCRBY KEY_NAME 增量
> - DECR KEY_NAME：key中的值自减一
> - DECRBY KEY_NAME
> - append key_name value：字符串拼接，追加至末尾，如果不存在，为其赋值
>

运用场景

- String通常用于保存单个字符串或JSON字符串数据
- 因为String是二进制安全的，所以可以把保密要求高的图片文件内容作为字符串来存储
- 计数器：常规Key-Value缓存应用，如微博数、粉丝数。INCR本身就具有原子性特性，所以不会有线程安全问题

#### hash命令

> - HSET key_name field value：为指定的key设定field和value
> - hmset key field value[field1,value1]
> - hget key field
> - hmget key field[field1]
> - hgetall key：返回hash表中所有字段和值
> - hkeys key：获取hash表所有字段
> - hlen key：获取hash表中的字段数量
> - hdel key field [field1]：删除一个或多个hash表的字段
> 	应用场景
>

Hash的应用场景，通常用来存储一个用户信息的对象数据。

- 相比于存储对象的string类型的json串，json串修改单个属性需要将整个值取出来。而hash不需要。
- 相比于多个key-value存储对象，hash节省了很多内存空间
- 如果hash的属性值被删除完，那么hash的key也会被redis删除

#### **list命令**



> - lpush key value1 [value2]
> - rpush key value1 [value2]
> - lpushx key value：从左侧插入值，如果list不存在，则不操作
> - rpushx key value：从右侧插入值，如果list不存在，则不操作
> - llen key：获取列表长度
> - lindex key index：获取指定索引的元素
> - lrange key start stop：获取列表指定范围的元素
> - lpop key ：从左侧移除第一个元素
> - prop key：移除列表最后一个元素
> - blpop key [key1] timeout：移除并获取列表第一个元素，如果列表没有元素会阻塞列表到等待超时或发现可弹出元素为止
> - brpop key [key1] timeout：移除并获取列表最后一个元素，如果列表没有元素会阻塞列表到等待超时或
> - 发现可弹出元素为止
> - ltrim key start stop ：对列表进行修改，让列表只保留指定区间的元素，不在指定区间的元素就会被删除
> - lset key index value ：指定索引的值
> - linsert key before|after world value：在列表元素前或则后插入元素
>

**list类似于Java中的LinkedList，其应用场景如下**

- 对数据大的集合数据删减
			列表显示、关注列表、粉丝列表、留言评价...分页、热点新闻等
- 任务队列
			list通常用来实现一个消息队列，而且可以确保先后顺序，不必像MySQL那样通过order by来排序

补充：

- rpoplpush list1 list2 移除list1最后一个元素，并将该元素添加到list2并返回此元素

- 用此命令可以实现订单下单流程、用户系统登录注册短信等。

#### set命令

> - sadd key value1[value2]：向集合添加成员
> - scard key：返回集合成员数
> - smembers key：返回集合中所有成员
> - sismember key member：判断memeber元素是否是集合key成员的成员
> - srandmember key [count]：返回集合中一个或多个随机数
> - srem key member1 [member2]：移除集合中一个或多个成员
> - spop key：移除并返回集合中的一个随机元素
> - smove source destination member：将member元素从source集合移动到destination集合
> - sdiff key1 [key2]：返回所有集合的差集
> - sdiffstore destination key1[key2]：返回给定所有集合的差集并存储在destination中
>

set的应用场景

- 对两个集合间的数据[计算]进行交集、并集、差集运算

- 以非常方便的实现如共同关注、共同喜好、二度好友等功能。对上面的所有集合操作，你还可以使用不同的命令选择将结果返回给客户端还是存储到一个新的集合中。

- 利用唯一性，可以统计访问网站的所有独立 IP

#### zset命令

> - zadd key score1 memeber1
> - zcard key ：获取集合中的元素数量
> - zcount key min max 计算在有序集合中指定区间分数的成员数
> - zcount key min max 计算在有序集合中指定区间分数的成员数
> - zrank key member：返回有序集合指定成员的索引
> - ZREVRANGE key start stop ：返回有序集中指定区间内的成员，通过索引，分数从高到底
> - ZREM key member [member …] 移除有序集合中的一个或多个成员
> - ZREMRANGEBYRANK key start stop 移除有序集合中给定的排名区间的所有成员(第一名是0)(低到高排序）
> - ZREMRANGEBYSCORE key min max 移除有序集合中给定的分数区间的所有成员
>

应用场景，常用于排行榜：

- 如推特可以以发表时间作为score来存储
- 存储成绩
- 还可以用zset来做带权重的队列，让重要的任务先执行

