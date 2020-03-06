## Chapter 02. 增删改查CURD

### 2.1 create/CREATE

功能：create创建数据库，数据表

```sql
create database 数据库名；      --小写
CREATE DATABASE 数据库名；--大写
CREATE TABLE table_name (column_name column_type);  --创建表，指定表(列名 列类型)；
```



### 2.2 insert into/INSERT INTO

功能：向表中插入一条或者多条记录，有两种形式一种需要指定列名，一种无需列名

```sql
-- 不指定列名
INSERT INTO table_name
VALUES (value1,value2,value3,...);
--指定列名
INSERT INTO tavle_name(colum1,colum2,colum3,...)
VALUES(value1,value2,value3,...)

```

### 2.3 select/SELECT

功能：查找语句，获得一个结果集。

```sql
SELECT column_name,column_name      --列名
FROM table_name                                              --表名
[WHERE Clause]                                                   --where条件语句 
[LIMIT N][ OFFSET M]；                                    --limit 限制返回数据量，offset查找偏移量
```

### 2.4 update/UPDATE

功能：更新语句，更改数据表中的数据

```sql
UPDATE table_name SET field=new_value1,field2=new_value2
[WHERE Clause]
```

### 2.5 delete/DELETE

功能：删除语句，删除数据表中数据

```sql
DELETE FROM table_name
[WHERE Clause]
```

## 