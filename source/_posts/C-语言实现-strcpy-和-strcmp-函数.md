---
title: C语言实现strcpy和strcmp函数
date: 2019-07-10 15:46:34
categories: 编程语言
tags: [C]
---
`strcpy`和`strcmp`是C语言标准函数库(C Standard library)提供的函数，在操作字符串的时候有着非常大的用处。
<!-- more -->

## strcpy

`strcpy`是C语言标准函数库(C Standard library)函数，其实现的功能是将源地址空间的字符串拷贝到目标字符串中。
头文件为 string.h，函数原型声明为：`char *strcpy(char *dest, const char *src)`，实现的功能为将从src地址开始且含有NULL结束符的字符串复制到以dest地址开始的内存空间。
The GNU C Library中 (源码下载方式为：git clone git://sourceware.org/git/glibc.git)，该函数的实现为：
``` C 
char *strcpy(char *dest, const char *src)
{
    return memcpy (dest, src, strlen (src) + 1);
}
```
`strcpy()`调用`memcpy()`从src处复制`strlen(src)+1`个字符到dest。这里有一个问题，那就是`strlen(src)`是否正确。我们知道，C字符串结束的标志是'\0'，我们在初始化一个char型数组的时候，若没有指定数组大小，则编译器会自动在最后添加一个'\0'，我们也可以手动添加'\0'。但是，有的情况下，会发生'\0'丢失的现象：
``` C
/* 以下情况会丢失'\0' */
char str[5] = {"hello"};  //数组长度不够加'\0'
char str[] = {'h','e','l','l','o'};  //每个字符单独引号括起来,编译器自动获取数组长度为5
/* 以下情况会自动添加'\0' */
char str[] = {"hello"};  //字符串赋值
char str[] = {'h','e','l','l','o','\0'};  //手动添加'\0'
char str[6] = {'h','e','l','l','o'};  //预留空位给'\0'
```
若参数 *src 指向的字符串是丢失'\0'的，那么`strlen(src)`会得到错误的结果，造成strcpy产生不可预知的后果。关于`strlen()`的原理，以及它在'\0'缺失的情况下如何获得字符数组长度，可以参考[博客](https://blog.csdn.net/hashmat/article/details/6054046)。

> 如果在一个需要字符串的地方（例如strlen函数的参数）使用了一个不是以NUL字节结尾的字符序列，会发生什么情况呢？strlen函数将无法知道NUL字节是没有的，所以它将继续进行查找，一个字符接一个字符，直到它发现一个NUL字节为止。或许它找了几百个字符才找到，而strlen函数的这个返回值从本质上说是一个随机数。或者，如果函数试图访问系统分配给这个程序以外的内存范围，程序就会崩溃。  ——《C与指针》

``` c
#include <stdio.h>
#include <string.h>
int main() {
    char str[5] = {"hello"};
    printf("strlen str = %lu\n", strlen(str));
    return 0;
}
```
这段代码如果用 gcc 编译，则输出为6，若用 clang 编译，则输出为5。gcc 编译器对代码做了优化，具体是什么优化，并不清楚，这是不好的，因为我们不知道它是怎么实现的，有没有什么隐患，我们能做的只有尽量避免使用。

若字符数组的长度不确定，会产生什么后果呢？
``` C
#include <stdio.h>
#include <string.h>
int main() {
    char arr[] = "123456789";
    char str[5] = {"hello"};
    printf("strlen str = %lu\n", strlen(str));
    char ds[10];
    strcpy(ds, str);
    printf("ds: %s#\n",ds);
    return 0;
```
这里出现了混乱，gcc显示strlen(str)=5，clang显示strlen(str)=14，clang的结果便于理解，因为若缺失'\0'，str就会和arr中的字符连在一起在内存栈区存放，先定义的在高地址，后定义的在低地址："hello123456789\0"，故长度为14，而gcc的结果，结合上一个实验，依然无法推断原因，所以说**依赖编译器优化的代码是危险的**。
至于strcpy的结果，clang显示为：hello123456789#，gcc显示为：hello#。
clang编译的代码显示，strcpy多拷贝了一部分内存数据，这就造成了很严重的问题。

> **警告**
> strlen根据'\0'判断字符串结束，那么恶意攻击者可以构造一个不包含'\0'的字符串，然后让数据写入数组之外的程序内存空间，从而进行破坏。


再来看下面的代码：
``` C
#include <stdio.h>
#include <string.h>
int main() {
    char s[] = "123456789";
    char d[] = "123";
    strcpy(d,s);
    printf("result d:%s  s:%s\n",d,s);
    return 0;
}
```
clang 编译后程序的运行结果为：result d:123456789  s:56789   
gcc 编译后程序运行结果为：result  d:123456789  s:123456789

clang 出现了奇怪的结果，我们来分析一下原因。首先，我们知道s和d都是在栈区的数据，s 在高地址，d 在低地址，我打印出了两者的地址，s：3318957810，d:3318957806。可以看到，内存是采用四字节对齐的，我们简单模拟一下内存中的数据：  
{% raw %}
<table>
    <tr>
        <td>地址</td>
        <td>1</td> <td>2</td> <td>3</td> <td>4</td>
        <td>5</td> <td>6</td> <td>7</td> <td>8</td>
        <td>9</td> <td>10</td> <td>11</td> <td>12</td>
        <td>13</td> <td>14</td>
    </tr>
    <tr>
        <td>数据</td>
        <td>1</td> <td>2</td> <td>3</td> <td>\0</td>
        <td>1</td> <td>2</td> <td>3</td> <td>4</td>
        <td>5</td> <td>6</td> <td>7</td> <td>8</td>
        <td>9</td> <td>\0</td>
    </tr>
    <tr>
        <td>字符串</td>
        <td colspan="4">d</td>
        <td colspan="10">s</td>
    </tr>
</table>
{% endraw %}

字符数组d从地址1开始，占4个字节，字符数组s从地址5开始，占10个字节。`strcpy`将s复制到d之后：

{% raw %}
<table>
    <tr>
        <td>地址</td>
        <td>1</td> <td>2</td> <td>3</td> <td>4</td>
        <td>5</td> <td>6</td> <td>7</td> <td>8</td>
        <td>9</td> <td>10</td> <td>11</td> <td>12</td>
        <td>13</td> <td>14</td>
    </tr>
    <tr>
        <td>数据</td>
        <td>1</td> <td>2</td> <td>3</td> <td>4</td>
        <td>5</td> <td>6</td> <td>7</td> <td>8</td>
        <td>9</td> <td>\0</td> <td>7</td> <td>8</td>
        <td>9</td> <td>\0</td>
    </tr>
    <tr>
        <td>字符串</td>
        <td colspan="4">d</td>
        <td colspan="10">s</td>
    </tr>
</table>
{% endraw %}

所以打印出来的d是123456789，而s是56789。这就造成了内存覆盖，可能产生无法预料的结果。  

> **警告：**  
> 程序员必须保证目标字符数组的空间足以容纳需要复制的字符串。如果字符串比数组长，多余的字符仍被复制，它们将覆盖原先存储于数组后面的内存空间的值。strcpy无法解决这个问题，因为它无法判断目标字符数组的长度。  ——《C与指针》



**总结一下strcpy的缺点：**  
1. strcpy 无法判断被复制的字符串在何处终止，可能会多复制一些内存数据到目标地址。从而产生内存溢出漏洞。
2. strcpy 从高地址向低地址拷贝的时候，不会检测目标地址是否有足够的空间来容纳字符串，容易产生内存覆盖。


关于strcpy的缺陷，用下面的例子来直观感受一下：
``` c
/* passwd.c */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Invalid params\n");
        exit(1);
    }
    bool check_result = false;
    char passwd[10];
    memset(passwd,0,10);
    strcpy(passwd, argv[1]);
    if (0 == strcmp("ubuntu", passwd)) {
        check_result = true;
    }
    if (check_result) {
        printf("Check passwd succ!\n");
    }
    else {
        printf("Check passwd failed!\n");
    }
    return 0;
}
```
这是一个简单的密码判别函数，将输入的密码保存到字符数组passwd中，然后同期望的密码"ubuntu"匹配。如果我们将代码编译完，然后运行`./passwd 123456789012345`，结果显示："Check passwd succ!"
而我们期望的是提示"Check passwd failed!"。尽管我们不知道密码是多少，但通过缓冲区溢出，绕过了密码的安全检验逻辑。
究其原因，是因为check_result和passwd两个变量同在栈区，check_out在高地址，passwd在低地址，strcpy的时候拷贝了超出目标地址passwd长度的字符，导致内存溢出，check_out变量被覆盖(被1234覆盖)，导致if条件判断为true。若输入为12345678900000，则不会，因为覆盖内存的是0000，if条件判断依然为false。



为了使字符数组的复制更加安全，一般情况下更推荐使用`strncpy`函数：
`char *strncpy(char *dst, char const *src, size_t len)`，添加了一个参数len来让程序员自己控制要复制的字符长度。
> 和strcpy一样，strncpy把源字符串的字符复制到目标数组。然而，它总是正好向dst写入len个字符。如果strlen( src )的值小于len，dst数组就用额外的NUL字节填充到len长度。如果strlen( src )的值大于或等于len，那么只有len个字符被复制到dst中。注意！它的结果将不会以NUL字节结尾。  ——《C和指针》

也就是说，strncpy函数解决了strcpy的第一个问题，它不会再多复制一些内容到目标地址中了，但是依然没有解决第二个问题，即内存覆盖的问题。


现在要实现一个安全的strcpy就要在基于strncpy的基础上防止内存覆盖：
``` C
char *my_strcpy(char *dest, const char *src, size_t len) {
    assert((dest != NULL) && (src != NULL));
    char *r = dest;
    if (dest < src && (src - dest) < len) {
        len = src - dest - 1;    //截断被复制的数据，并预留一字节给\0
    }
    while (len > 0) {
        *r++ = *src++;
        len--;
    }
    *r++ = '\0';
    return dest;
}
```


## strcmp

待续

 

