## Chapter 04 SQL函数

### 4.1 SQL Aggregate 函数

SQL Aggregate 函数计算从列中取得的值，返回一个单一的值。

| 函数名  | 功能                 |
| ------- | -------------------- |
| AVG()   | 返回平均值           |
| COUNT() | 返回行数             |
| FIRST() | 返回第一个记录的值   |
| LAST()  | 返回最后一个记录的值 |
| MAX()   | 返回最大值           |
| MIN()   | 返回最小值           |
| SUM()   | 返回总和             |

### 4.2 SQL Scalar 函数

SQL Scalar 函数基于输入值，返回一个单一的值。

| 函数名                  | 功能                                     |
| ----------------------- | ---------------------------------------- |
| UCASE()                 | 将某个字段转换为大写                     |
| LCASE()                 | 将某个字段转换为小写                     |
| MID()                   | -从某个文本字段提取字符，MySql 中使用    |
| SubString(字段，1，end) | 从某个文本字段提取字符                   |
| LEN()                   | 返回某个文本字段的长度                   |
| ROUND()                 | 对某个数值字段进行指定小数位数的四舍五入 |
| NOW()                   | 返回当前的系统日期和时间                 |
| FORMAT()                | 格式化某个字段的显示方式                 |

### 4.3 GROUP BY

GROUP BY 语句用于结合聚合函数，根据一个或多个列对结果集进行分组

```sql
SELECT column_name, aggregate_function(column_name)
FROM table_name
WHERE column_name operator value
GROUP BY column_name;
```

### 4.4 HAVING 语法

在 SQL 中增加 HAVING 子句原因是，WHERE 关键字无法与聚合函数一起使用。

HAVING 子句可以让我们筛选分组后的各组数据。

```sql
SELECT column_name, aggregate_function(column_name)
FROM table_name
WHERE column_name operator value
GROUP BY column_name
HAVING aggregate_function(column_name) operator value;
```

### 4.5 EXISTS

EXISTS 运算符用于判断查询子句是否有记录，如果有一条或多条记录存在返回 True，否则返回 False。

```sql
SELECT column_name(s)
FROM table_name
WHERE EXISTS
(SELECT column_name FROM table_name WHERE condition);
```

## 