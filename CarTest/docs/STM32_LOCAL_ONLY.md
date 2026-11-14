# VS Code STM32启动检查阻塞处理

本工程使用已安装的本地STM32工具链。现场观察到 `cube --json --cwd ... bundle install --no --project` 阻塞插件激活，结束该检查后调试插件继续加载。

当前工作区 `.vscode/settings.json` 已启用官方设置：

```json
{
  "stm32cube-ide-core.LocalOnly": true,
  "stm32cube-ide-core.synchronizePdscRepositoriesOnStartup": false
}
```

Local Only模式由插件提供，跳过联网请求并关闭在线工具包、芯片包更新。设置仅作用于本工程，不修改系统网络代理，也不删除工具包。编译器、GDB、ST-Link GDB Server、CubeProgrammer和Ninja的现有路径均已检查。

下次启动VS Code时使用此配置。若当前窗口仍有旧检查，可结束该窗口的调试后执行“开发人员：重新加载窗口”。配置写入与本地构建检查不等于已验证重启后不再阻塞；还需实际重新加载窗口确认。

需要安装新工具包或更新芯片包时，临时关闭LocalOnly，完成更新后重新开启。本次仅调整编辑器配置，不改变车辆固件逻辑，无需因此重新烧录。
