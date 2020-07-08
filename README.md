## robust-vslam 初级版本
Tips:The basic code framework from robust-vslam comes from [slambook2](https://github.com/gaoxiang12/slambook2)

## 存在问题

- kitti数据集下，定位还不错。但是在楼下的实验室环境，定位结果却不理想。

- 大范围，纹理变化较为明显的环境还不错；纹理变化不明显，小范围的室内定位却不行。

- 下一版本要实现楼下室内实验室的视觉定位

## 可能的改进点

- 特征点提取要均匀，识别性较好。
- 特征匹配要尽可能多地排除误匹配，要多加约束。特别是对于纹理单一的环境，比如标定板，必须要加入一个几何约束。
- 三角化时候，大深度出现的问题。
- 跟踪的综合设计



