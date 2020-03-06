## Chapter 03 高级关键字

### 3.1 DISTINCT

功能：查询中可能有重复记录，distinct去除重复记录

```sql
SELECT DISTINCT 列名称 FROM 表名称
```

### 3.2 Order By

功能：排序语句，ORDER BY 语句用于根据指定的列对结果集进行排序,可以有优先级，ORDER BY 语句默认按照升序对记录进行排序。若需要降序对记录进行排序，可以使用 DESC 关键字。

```sql
SELECT * FROM table_name ORDER BY column_name1,column_name2,...
```

### 3.3 Like

功能：LIKE 操作符用于在 WHERE 子句中搜索列中的指定模式。可以使用正则表达式

```sql
SELECT column_name(s)
FROM table_name
WHERE column_name LIKE pattern     -- pattern为正则表达式
```

通配符：

| 符号                    | 解释                       |
| :---------------------- | :------------------------- |
| %                       | 替代一个或多个字符         |
| _                       | 仅替代一个字符             |
| [charlist]              | 字符列中的任何单一字符     |
| [^charlist]，[!charlis] | 不在字符列中的任何单一字符 |

### 3.4 IN

功能：用与WHERE语句中，规定多个值

```sql
SELECT * FROM table_name 
WHERE column_name IN ("name1","name2")

```

### 3.5 BETWEEN

功能：操作符 BETWEEN ... AND 会选取介于两个值之间的数据范围。这些值可以是数值、文本或者日期。

```sql
SELECT column_name(s)
FROM table_name
WHERE column_name
BETWEEN value1 AND value2
```

### 3.6 Alias

功能：设置别名

给行设置别名

```sql
SELECT column_name(s)
FROM table_name
AS alias_name

```

给列设置别名

```sql
SELECT column_name AS alias_name
FROM table_name
```

### 3.7 JOIN连接关键字

功能：SQL JOIN 子句用于把来自两个或多个表的行结合起来，基于这些表之间的共同字段。交运算

- INNER JOIN

	功能：INNER JOIN 关键字在表中存在至少一个匹配时返回行。

```sql
SELECT column_name(s)
FROM table1
INNER JOIN table2
ON table1.column_name=table2.column_name;
```

- JOIN

	功能：与INNER JOIN关键字一样，在表中存在至少一个匹配时返回行。

```sql
SELECT column_name(s)
FROM table1
JOIN table2
ON table1.column_name=table2.column_name;   --on表条件
```

- LEFT JOIN/RIGHT JOIN

	功能： LEFT JOIN关键字会从左表 (table_name 1) 那里返回所有的行，即使在右表 (table_name 2) 中没有匹配的行;RIGHT JOIN 关键字会右表 (table_name 2) 那里返回所有的行，即使在左表 (table_name 1) 中没有匹配的行。

```sql
SELECT column_name(s)
FROM table1
LEFT JOIN table2
ON table1.column_name=table2.column_name;
```

或

```sql
SELECT column_name(s)
FROM table1
LEFT OUTER JOIN table2
ON table1.column_name=table2.column_name;
```

- FULL JOIN

	功能：FULL OUTER JOIN 关键字只要左表（table1）和右表（table2）其中一个表中存在匹配，则返回行。FULL OUTER JOIN 关键字结合了 LEFT JOIN 和 RIGHT JOIN 的结果。

```sql
SELECT column_name(s)
FROM table1
FULL OUTER JOIN table2
ON table1.column_name=table2.column_name;
```

### 3.8 Union合并关键字

功能：UNION 操作符用于合并两个或多个 SELECT 语句的结果集。

默认地，UNION 操作符选取不同的值。如果允许重复的值，请使用 UNION ALL。

```sql
SELECT column_name(s) FROM table1
UNION
SELECT column_name(s) FROM table2;
```

```sql
SELECT column_name(s) FROM table1
UNION ALL
SELECT column_name(s) FROM table2;

```

### 3.9 INSERT INTO SELECT 

功能：INSERT INTO SELECT 语句从一个表复制数据，然后把数据插入到一个已存在的表中。目标表中任何已存在的行都不会受影响。

从一个表中复制所有的列插入到另一个已存在的表中：

```sql
INSERT INTO table2
SELECT * FROM table1;
```

复制希望的列插入到另一个已存在的表中：

```sql
INSERT INTO table2
(column_name(s))
SELECT column_name(s)
FROM table1;
```

### 3.10 Constraints关键字

功能：约束关键字，如果存在违反约束的数据行为，行为会被约束终止。约束可以在创建表时规定（通过 CREATE TABLE 语句），或者在表创建之后规定（通过 ALTER TABLE 语句）。

- **NOT NULL** - 指示某列不能存储 NULL 值。
- **UNIQUE** - 保证某列的每行必须有唯一的值。
- **PRIMARY KEY** - NOT NULL 和 UNIQUE 的结合。确保某列（或两个列多个列的结合）有唯一标识，有助于更容易更快速地找到表中的一个特定的记录。
- **FOREIGN KEY** - 保证一个表中的数据匹配另一个表中的值的参照完整性。
- **CHECK** - 保证列中的值符合指定的条件。
- **DEFAULT** - 规定没有给列赋值时的默认值。

### 3.11 ALTER TABLE

功能：ALTER TABLE 语句用于在已有的表中添加、删除或修改列。

向表中添加列：

```sql
ALTER TABLE table_name
ADD column_name datatype
```

删除表中的列：

```sql
ALTER TABLE table_name
DROP COLUMN column_name
```

更改表中列类型：

```sql
ALTER TABLE table_name
MODIFY COLUMN column_name datatype
```

### 3.12 CREATE INDEX

功能：CREATE INDEX 语句用于在表中创建索引。在不读取整个表的情况下，索引使数据库应用程序可以更快地查找数据。

创建一个简单的索引,允许使用重复的值：

```sql
CREATE INDEX index_name
ON table_name (column_name)
```

在表上创建一个唯一的索引。不允许使用重复的值：唯一的索引意味着两个行不能拥有相同的索引值:

```sql
CREATE UNIQUE INDEX index_name
ON table_name (column_name)
```

### 3.13 DROP

功能：DROP 语句，撤销（删除）索引、表和数据库

删除索引（mysql）：

```sql
ALTER TABLE table_name DROP INDEX index_name
```

删除表：

```sql
DROP TABLE table_name

```

删除数据库：

```sql
DROP DATABASE database_name
```

仅删除表中数据而不删除表本身，TRUNCATE TABLE 

```sq
TRUNCATE TABLE table_name

```

### 3.14 Auto-increment

功能：Auto-increment 会在新记录插入表中时生成一个唯一的数字。

### 3.15 WHERE与ON

- on条件是在生成临时表时使用的条件，它不管on中的条件是否为真，都会返回左边表中的记录。
- where条件是在临时表生成好后，再对临时表进行过滤的条件。这时已经没有left join的含义（必须返回左边表的记录）了，条件不为真的就全部过滤掉。

