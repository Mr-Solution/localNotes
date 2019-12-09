---
title: 剑指 offer 读书笔记：编程语言篇
date: 2019-06-22 15:02:09
categories: 编程语言
tags: [读书笔记]
---

C/C++ 的字符串、数组、指针、结构体、堆栈、static关键字、const关键字，是非常重要且经常出错的知识点。

<!--more-->
# C/C++

## 1. 基础知识

### 1.1 C 字符串和 C++ 字符串

- C 语言是没有字符串的，C 风格字符串是以一维字符数组的形式存储，并以字符'\0'终止。
- C++ 中引入了 string 类，字符串在 C ++ 中用一个 string 实例表示。


### 1.2 sizeof()

#### 数组

``` C
#include <stdio.h>
int main() {
    char *p = "abcdef";
    char a[] = "abcdef";
    printf("sizeof(p) = %lu\n", sizeof(p));
    printf("sizeof(a) = %lu\n", sizeof(a));
    return 0;
}
```
输出`sizeof(p) = 8`，`sizeof(a) = 7`。`sizof(p)`输出的是指针 p 的大小，指针的大小同系统的寻址能力有关，在64位系统中，一个指针为64位，等于8字节，所以 sizeof(p)=8。a 是一个数组名，编译器用数组名来记录数组的属性，比如数组的大小等。数组名只有在表达式中使用时，编译器才会为它产生一个指针常量，但是有两个例外——`sizeof` 操作符和单目操作符 `&`。`sizeof`返回整个数组的长度，而不是指向数组的指针的长度。取一个数组名的地址产生的是一个指向数组的指针，而不是指向某个指针常量值的指针。  
一定要记住：**数组名并不等价于指针**。

**· 问：sizeof 是如何知道数组大小的？**  
**·** 答：sizeof(arr)不是程序执行到这里的时候才去求值的，并且sizeof(arr)计算的是*数组所占字节数，并非数组长度*，数组长度 = sizeof(arr)/sizeof(类型)。例如，定义一个数组 arr，`int arr[10]`，那么`sizeof(arr)`的结果为 40，而`sizeof(arr)/sizeof(int)`的结果为 10。`sizeof`关键字是在编译阶段处理的，在程序运行之前，sizeof(arr)的结果就已经被计算好并保存在.out文件中。

#### 结构体

结构体的地址对齐有两条原则：
1. 结构体中成员的偏移量必须是成员大小的整数倍；
2. 结构体大小必须是所有成员大小的整数倍，即所有成员大小的公倍数。

``` c
typedef struct {
    char c;
    int i;
} S1;
typedef struct {
    char c1;
    S1 s1;
} S2;
...
printf("sizeof(S1) = %lu\n", sizeof(S1));
printf("sizeof(S2) = %lu\n", sizeof(S2));
```
运行结果为：`sizeof(S1) = 8`，`sizeof(S2) = 12`。char 占 1 个字节，int 占 4 个字节，在对齐的时候，c 的偏移量为 0，i 的偏移量为 4，S1 的大小为 8。对于嵌套的结构体 S2，需要将其展开。两条原则变为：
1. 展开后的结构体的第一个成员的偏移量应当是被展开的结构体中最大的成员的整数倍。
2. 结构体大小必须是所有成员(展开后的成员)大小的整数倍。  
 
这里 S1 中最大的类型为 int，占4字节，所以，s1.c 的偏移量为4，s1.i 的偏移量为8，S2 的大小为12。

对空的结构体求 sizeof，结果与编译器有关，clang 和 gcc 均返回0。结构体的对齐方式还会收到预编译指令`#pragma pack()`的影响，具体内容看 <a href="#struct">2.1</a> 节。


### 1.3 strlen()

表面上看，`strlen`和`sizeof`都可以求字符串的长度，但二者却存在着本质的区别。
1. `sizeof`是一个运算符，`strlen`是一个函数。
2. `sizeof`返回操作对象所占的内存字节数；`strlen`返回字符串的长度(字符的个数，并不等于内存字节数)，不包括结束字符('\0' )。
3. `sizeof`的结果在编译阶段生成；`strlen`的结果在运行阶段产生。

``` c
char str[] = "abc";
printf("sizeof(str) = %lu\n", sizeof(str));
printf("strlen(str) = %lu\n", strlen(str));
```
输出分别为4和3。

### 1.4 C++中字符数组和指针的区别

参考[博客](https://www.cnblogs.com/Kernel001/p/8259398.html)。


第一个例子：
``` c
char str1[] = "abc";
char str2[] = "abc";
printf("%d\n", str1 == str2);
```
这段代码最终会输出结果 0。因为，"abc"是一个常量，保存在常量存储区，str1 是一个数组变量，保存在栈区，语句`char str1[] = "abc"`的意思是，在栈区申请大小为 4 的空间，保存"abc"(字符数组结尾默认添加'\0')。同理，`char str2[] = "abc"`也申请了一块栈区的内存。即 str1 和 str2 存放在不同的位置，在表达式`str1 == str2`中，str1 和 str2 都是指针，分布指向两个数组的起始地址，故 str1 == str2 会返回 false。  
在使用 clang 编译代码的时候，编译器会提示：
> warning: array comparison always evaluates to false

就是告诉用户，数组内容虽然一样，但是数组名作为指针，指向的是不同的内存区域。gcc 则缺少相应的提示。

第二个例子：
``` c
const char *str3 = "abc";
const char *str4 = "abc";
printf("%d\n", str3 == str4);

const char str5[] = "abc";
const char str6[] = "abc";
printf("%d\n", str5 == str6);
```
第一个`printf`会输出1，第二个`printf`会输出0。这是因为，使用`const`定义的变量一般是不分配内存，而是保存在符号表中的，即指针 str3 和 str4 都是保存在符号表上，指向常量存储区中的常量"abc"，所以 str3 == str4。但是，对于 const 数组来说，因为系统不确定符号表是否有足够的空间来存放数组，所以还是会在栈区分配内存，所以 str5 指向的是栈区的"abc"，str5 不等于 str6。



### 1.5 const


### 1.6 static

## 2. 内存管理

### 2.1 <a id="struct">struct 对齐</a>



### 2.2 堆和栈

### new&delete

## 3. 常见的一些函数

### 3.1 gets()

### 3.2 strcpy()

### 3.3 malloc() & free()

### 3.4 new() & delete()


**Q&A SUMMARIZE**
+ Q：对一个空的类型求 sizeof，得到的结果是多少？  
  A：~~1。空类型的实例中不包含任何信息，本来求 sizeof 是 0，但是当我们声明该类型的实例的时候，它必须在内存中占有一定的空间，否则无法使用这些实例。具体占用多少内存，由编译器决定。~~  
  A：0。gcc 和 clang 的编译结果都是 0。
+ Q：如果在空类型中添加一个构造函数和析构函数，再求 sizeof，结果是多少？
  A：1。调用构造函数和析构函数只需要知道函数的地址即可，而这些函数的地址只与类型相关，而与类型的实例无关，编译器也不会因为这两个函数而在实例内添加任何额外的信息。   
  Q：如果把析构函数标记为虚函数呢？
  A：C++ 编译器发现类型中有虚函数，就会为该类型生成虚函数表，并在该类型的每一个实例中添加一个指向虚函数表的指针。在32位的机器上，一个指针占 4 字节，求 sizeof 结果为 4。
