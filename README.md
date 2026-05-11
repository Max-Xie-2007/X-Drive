# 项目说明书

## 角度约定

- 内部角度都使用逆时针为正方向，例如`robot-config.h`中`Inertial`的配置`inertial(left)`，`myPosition::heading_`，`Vector::angle()`以及`Vector::rotate(angle)`。
- 角度函数返回的角度值统一为**0～360而非-180～180**，因此在计算转向误差时须额外判断是否大于180（若是，则减去360）

## 代码格式

- 在VSCode设置中搜索Format并在`format style`里粘贴以下文本：

```{BasedOnStyle: LLVM, ColumnLimit: 90, UseTab: Never, IndentWidth: 4, TabWidth: 4, BreakBeforeBraces: Attach, AllowShortIfStatementsOnASingleLine: true, AccessModifierOffset: -2, NamespaceIndentation: All, FixNamespaceComments: false, PointerAlignment: Left, IndentCaseLabels: true}```

- 开启`format on save`等功能

## 正反转配置规则

- 将所有底盘电机设置成使车身逆时针转为正方向

## 命名规则

- 私有类成员变量使用**蛇形+下划线后缀命名法**
- 函数传参及普通变量使用**蛇形命名法**
- 所有方法名都用**小驼峰命名法**
- 机器人硬件名使用**首字母大写+蛇形命名法**

### `motion.cpp` 变量命名补充

- 目标状态统一使用 `target_*` 前缀，例如：`target_position`、`target_heading`
- 当前状态统一使用 `current_*` 前缀，例如：`current_position`、`current_heading`
- 误差统一使用 `*_error` 后缀，例如：`translational_error`、`angular_error`
- 路径相关误差统一使用 `distance_error`
- 相对朝向偏移统一使用 `heading_offset`
- 终点目标朝向统一使用 `terminal_target_heading`

### 日志命名补充

- 终端输出字段名必须与代码变量语义一致
- 推荐输出键名：`target_position`、`target_heading`、`current_position`、`current_heading`、`elapsed_time`
- 若改动变量命名，需同步修改对应日志文本，保持代码与日志一致

## 其他事项

- 使用常量时从`params.h`中调用，不要直接写数字（尤其是线程刷新）
