# GR5xxx-Multi-Peripheral-Multi-Central-Demo

## 应用说明
GR5xxx-Multi-Peripheral-Multi-Central-Demo 主要是为多主多从应用开发，其抽象链路交互和状态维护，独立解耦各个链路，用户创建链路instance即可，方便移植开发，是用户进关注应用逻辑开发即可


## 运行展示
* 先clone GR551x SDK到本地工作区 
* 将Demo工程拷贝到 ${GR551x.SDK}\projects\ble\ble_multi_role 目录下
* 使用 keil环境构建编译下载到 GR551x SK开发板体验即可
* 如果有Goodix其他的SK板, 可以轻松移植到对应的SDK下

## 注意事项
* 运行于其他GR5xxx平台，需更换对应平台的custom_config.h
* 运行于其他GR5xxx平台存在编译错误，稍微对应修改即可，不会太复杂，自行修改