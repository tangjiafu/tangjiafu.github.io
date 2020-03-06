## Chapter 06. JDBC三层架构

### 6.1 JDBC架构

JDBC（**J**ava **D**ata**B**ase **C**onnectivity）是Java和数据库之间的一个桥梁，是一个规范而不是一个实现，能够执行SQL语句。它由一组用Java语言编写的类和接口组成。各种不同类型的数据库都有相应的实现。

分为双层架构与三层架构。

- 双层架构：客户端直接访问数据库，C/S配置
- 三层架构：客户端与数据库含有一个中间层，不直接访问数据库。分为表示层，业务逻辑层，数据访问层。

### 6.2 JDBC基本流程

1. 装载相应的数据库的JDBC驱动并进行初始化

2. 建立JDBC和数据库之间的Connection连接

3. 创建Statement或者PreparedStatement接口，执行SQL语句

4. 处理和显示结果

5. 释放资源

``` java
package com.tang.jdbc;
import java.sql.*;

public class demoJDBC {
    private static final String JDBC_DRIVER = "com.mysql.cj.jdbc.Driver";
    private static final String DB_URL = "jdbc:mysql://localhost/lagTest";
    private static final String USER = "root";
    private static final String PASSWORD = "root";

    private static void helloJDBC() throws ClassNotFoundException {
        Connection conn = null;
        Statement stmt = null;
        ResultSet rs = null;
        //1.装载驱动程序
        Class.forName(JDBC_DRIVER); //Class.forName 方法的作用，就是初始化给定的类
        //2.建立数据库链接
        try {
            conn = DriverManager.getConnection(DB_URL, USER, PASSWORD);
            //3.创建statement
            stmt = conn.createStatement();
            //4 执行SQL语句
            rs = stmt.executeQuery("select name from hello");
            //5.遍历 获取执行结果
            while (rs.next()) {
                System.out.println("hello: " + rs.getString("name"));
            }
        } catch (SQLException e) {
            //异常处理
            e.printStackTrace();
        } finally {
            //5.清理环境
            try {
                if (conn != null) { //为了防止java空指针异常，因为conn有可能是空指针
                    conn.close();
                }
                if (stmt != null) {
                    stmt.close();
                }
                if (rs != null) {
                    rs.close();
                }
            } catch (SQLException e) {
                e.printStackTrace();
            }
        }
    }

    public static void main(String[] args) throws ClassNotFoundException {
        helloJDBC();
    }
}
```



### 6.3 JDBC事务



``` java
 try {
            conn = dataSource.getConnection();
            conn.setAutoCommit(false);//不自动commit事务
            //3.创建statement
            ptmt = conn.prepareStatement("update hello set password = ? where name = ?");
            ptmt.setString(1, "ok13456");
            ptmt.setString(2, "zhangsan");
            ptmt.execute();
            sp = conn.setSavepoint(); //事务回滚点
            ptmt.setString(1, "ok13456");
            ptmt.setString(2, "lisi");
            ptmt.execute();
            conn.commit(); //事务提交
        } catch (SQLException e) {
            //异常处理
            if (conn != null) {
                try {
                    conn.rollback(sp); //如果出现sql异常，事务回滚到sp标记点，保证ASID特性
                } catch (SQLException e1) {
                    e1.printStackTrace();
                }
            }
        } finally {
            //5.清理环境
            try {
                if (conn != null) { //为了防止java空指针异常，因为conn有可能是空指针
                    conn.close();
                }
                if (ptmt != null) {
                    ptmt.close();
                }

            } catch (SQLException e) {
                e.printStackTrace();
            }
        }
```



### 6.4 JDBC连接池与限流

``` java
package com.tang.jdbc;

import org.apache.commons.dbcp2.BasicDataSource;

import java.sql.Connection;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;

/**
 * 连接池
 */
public class poolJDBC {
    private static BasicDataSource ds = new BasicDataSource();

    private static final String JDBC_DRIVER = "com.mysql.cj.jdbc.Driver";
    private static final String DB_URL = "jdbc:mysql://localhost/lagTest";
    private static final String USER = "root";
    private static final String PASSWORD = "root";

    /**
     * 数据库连接池初始化
     */
    private static void dbpoolInit() {
        ds.setUrl(DB_URL);
        ds.setDriverClassName(JDBC_DRIVER);
        ds.setUsername(USER);
        ds.setPassword(PASSWORD);
    }

    private void dbpoolITest() throws SQLException {
        Connection conn = null;
        Statement stmt = null;
        ResultSet rs = null;
        try {
            conn = ds.getConnection();
            stmt = conn.createStatement();
            String sql = "select * from hello";
            rs = stmt.executeQuery(sql);
            while (rs.next()) {
                System.out.println(rs.getString("name"));
            }
        } catch (SQLException e) {
            e.printStackTrace();
        } finally {
            if (conn != null) conn.close(); //注意，这里关闭，是归还到了连接池
            if (stmt != null) conn.close();
            if (rs != null) rs.close();
        }
    }

    public static void main(String[] args) throws SQLException {
        dbpoolInit();
        new poolJDBC().dbpoolITest();
    }
}
```

### 6.5 批处理SQL语句与io流

``` java
package com.tang.jdbc;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.HashSet;
import java.util.Set;

/**
 * 批处理
 */
public class BatchJDBC {
    private static final String JDBC_DRIVER = "com.mysql.cj.jdbc.Driver";
    private static final String DB_URL = "jdbc:mysql://localhost/lagTest";
    private static final String USER = "root";
    private static final String PASSWORD = "root";

    private static void batchTestJDBC(Set<String> names) throws ClassNotFoundException {
        Connection conn = null;
        Statement stmt = null;
        // 1 load databases
        Class.forName(JDBC_DRIVER);
        // 2 connect
        try {
            conn = DriverManager.getConnection(DB_URL, USER, PASSWORD);
            //3. create Statement
            stmt = conn.createStatement();
            //4. make sql
            for (String usrName : names) {
                stmt.addBatch(String.format("insert into hello(name) values(\" %s\")", usrName));
            }
            stmt.executeBatch();
            stmt.clearBatch();
        } catch (SQLException e) {
            e.printStackTrace();
        } finally {
            try {
                if (conn != null) { //为了防止java空指针异常，因为conn有可能是空指针
                    conn.close();
                }
                if (stmt != null) {
                    stmt.close();
                }
            } catch (SQLException e) {
                e.printStackTrace();
            }
        }
    }

    public static void main(String[] args) throws ClassNotFoundException {
        Set<String> names = new HashSet<>();
        names.add("zhang");
        names.add("li");
        names.add("wang");
        batchTestJDBC(names);
    }
}
```



``` java
conn = DriverManager.getConnection(DB_URL, USER, PASSWORD);
            //3.创建statement
 ptmt = conn.createStatement();
            //4 执行SQL语句
 rs = ptmt.executeQuery("select * from hello");
            //5.遍历 获取执行结果
 while (rs.next()) {
//              System.out.println("hello: " + rs.getString("name"));
                InputStream in = rs.getBinaryStream("name");
                //6.将对象写入文件
                File f = new File("./test.txt");
                OutputStream out;
                out = new FileOutputStream(f);
                int temp;
                while ((temp = in.read()) != -1) {
                    System.out.println(in.read());
                    out.write(temp);
                }
                in.close();
                out.close();
 }
```



### 6.4 数据库SQL安全

SQL注入攻击，避免拼接字符串成SQL语句

``` java
package com.tang.jdbc;

import java.sql.*;

/**
 * 模拟sql注入攻击
 */
public class sqlSafe {
    private static final String JDBC_DRIVER = "com.mysql.cj.jdbc.Driver";
    private static final String DB_URL = "jdbc:mysql://localhost/lagTest";
    private static final String USER = "root";
    private static final String PASSWORD = "root";

    private static boolean login(String User, String passWord) throws ClassNotFoundException {
        Connection conn = null;
        PreparedStatement ptmt = null;
        ResultSet rs = null;
        boolean flag = false;
        //1.装载驱动程序
        Class.forName(JDBC_DRIVER); //Class.forName 方法的作用，就是初始化给定的类
        //2.建立数据库链接
        try {
            conn = DriverManager.getConnection(DB_URL, USER, PASSWORD);
            //3.创建statement
            ptmt = conn.prepareStatement("select *from hello where name=? and password=?");
            ptmt.setString(1, User);
            ptmt.setString(2, passWord);
            rs = ptmt.executeQuery();
            //遍历 获取执行结果
            while (rs.next()) {
                System.out.println("hello: " + rs.getString("name"));
                flag = true;
            }
        } catch (SQLException e) {
            //异常处理
            e.printStackTrace();
        } finally {
            //5.清理环境
            try {
                if (conn != null) { //为了防止java空指针异常，因为conn有可能是空指针
                    conn.close();
                }
                if (ptmt != null) {
                    ptmt.close();
                }
                if (rs != null) {
                    rs.close();
                }

            } catch (SQLException e) {
                e.printStackTrace();
            }

        }
        return flag;
    }

    public static void main(String[] args) throws ClassNotFoundException {
        boolean loginTest = login("tang';-- ", "8888");
        System.out.println(loginTest);
    }
}
```

