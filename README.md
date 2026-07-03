# 项目简要

## 前情提要
因为老师给了控制小车的MecanumDriver类，所以我们控制舵机需要使用该类的方法。然而该类
`analogWrite(pin1, constrain(abs(speed), 0, 100));`等相关代码使得小车舵机速度有上限，实则
可以更快，于是我把该类的相关代码进行了修改，使得小车舵机可以更快地运行。
```C++
analogWrite(pin1, constrain(abs(speed), 0, 1000));
//使得小车舵机速度上限为1000，远超原先100的上限
```


## 项目文件结构
```bash
OurCodes/
├── README.md                          # 项目文档
├── LICENSE.txt                        # 许可证文件
├── .gitattributes                    # Git属性配置
├── .gitignore                        # Git忽略规则
├── file_structure.txt                # 文件结构说明
│
├── 八向避障/                          # 避障导航主程序
│   └── exp1/
│       ├── exp1.ino                  # ⭐ 主程序入口
│       ├── MecanumDriver.h           # 电机驱动接口
│       └── MecanumDriver.cpp         # 电机驱动实现
│
├── 回字型/                            # 回字形路径测试
│   └── exp2/
│       ├── exp2.ino                  # 回字形测试程序
│       ├── MecanumDriver.h           
│       └── MecanumDriver.cpp         
│
├── 八字形/                            # 八字形路径测试
│   └── exp3/
│       ├── exp3.ino                  # 八字形测试程序
│       ├── MecanumDriver.h           
│       └── MecanumDriver.cpp         
│
├── out/                               # 编译输出目录
│   └── build/
│       └── x64-debug/                # Debug构建文件
│
└── .vs/                               # Visual Studio配置
    └── ...
```