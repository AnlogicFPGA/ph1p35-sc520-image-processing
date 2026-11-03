# ph1p35-sc520-image-processing

本仓库用于项目成果展示。团队原创代码保留全部权利。除适用法律及平台条款允许的情形外，未经书面授权，不得复制、修改、再分发，或用于商业项目及其他竞赛。第三方代码、厂商IP及相关资料遵循其原有授权。



\# 多模式实时数字图像处理终端



基于安路PH1P35 FPGA与HX1P35A开发板的多模式实时数字图像处理终端，采用SC520摄像头、DDR2缓存及1280×720@60Hz HDMI输出，支持图像增强、多模式处理、目标检测与坐标标注、运动计数及二维云台跟随。



\## 工程内容



\- `project/td\_project/camera\_to\_dsi\_display.al`：Tang Dynasty工程入口，顶层模块为`design\_top\_wrapper`。

\- `project/user\_source`：SC520CS采集、图像处理、DDR2缓存、HDMI显示、目标检测、运动计数及双轴云台控制的RTL、IP和约束。

\- `project/td\_project/al\_ip`：工程引用的安路IP配置及生成文件。

\- `project/sim`：OSD和云台参考点的仿真Testbench。

\- `bitstream/SC520\_VIDEO\_GIMBAL\_FINAL.bit`：可供下载的位流，SHA-256为`c47e2a0aa8ca4f78058ec829efb36e64f915c7de69810f457add2a8dba57dc7d`。

\- `assets/anlogic/official\_logo.svg`：安路官方标识源文件。

\- `SHA256SUMS.txt`：除清单自身外的文件校验值。



\## 环境与运行



目标器件为PH1P35MDG324，速度等级-3；开发软件为Tang Dynasty 6.2.168116；摄像头为SC520CS，HDMI规格为1280×720@60Hz。将目录解压到较短英文路径后，用TD打开上述工程入口，或使用附带位流下载演示。重新生成位流时，应核对器件、引脚、IP、资源和最终Setup/Hold时序。



系统提供COLOR、GRAY、BINARY、EDGE、CARTOON、TRACK和OBJECT模式。OBJECT模式使用有效目标观测控制云台；退出该模式停止PWM输出。目标选择、曝光/白平衡、检测及运动计数使用工程中的现有控制逻辑。



\## 云台与激光配置



1280×720图像坐标的原点为左上角，x向右、y向下。顶层的`GIMBAL\_REF\_X=684`、`GIMBAL\_REF\_Y=264`为跟随参考点，用于补偿摄像头与激光器的相对位置。目标稳定位置会靠近该参考点。实际照射偏差与工作距离、安装和目标表面有关；距离或安装变化时应重新标定。保留原有停止/启动死区及抑振控制。



双轴PWM封装脚为C18和D15，LVCMOS33。不同板卡修订的J1/J2标记可能不同，接线应对照实际板卡手册和丝印。舵机供电与GPIO信号须区分并正确共地，5V电源不得接入3.3V GPIO。激光使用电源脚和GND供电，不由GPIO直接驱动。



颜色检测取决于摄像头标定、光照和目标/背景对比。`LAT`为链路内部延迟，不包含完整的曝光及显示器响应时间；帧序号不等于帧率。



\## 参考点仿真



在ModelSim中将工作目录设为`project/sim`，执行`do tb\_gimbal\_reference.do`。该测试检查参考点保持、修正方向、死区及无效观测/模式退出行为。



\## 第三方文件



工程中的厂商IP、参考代码和工具生成文件保留原始版权、作者署名及使用范围声明。安路标识归安路科技所有，源文件来自`https://www.anlogic.com/Public/Uploads/uploadfile/images/20230414/logo.svg`。相关文件的使用及再分发遵循其原有授权。





