DTSI修改规则：
1.QCOM 路径下的所有修改保持原生态
2.平台共性问题修改放到bengal-rum.dtsi，或者新建dtsi文件并包含到bengal-rum.dtsi中，注意平台首个项目，默认
都放到bengal-rum.dtsi中。
3.项目之间的差异，创建自己项目的dtsi，例如bengal-rum-20241.dtsi