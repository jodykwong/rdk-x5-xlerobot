# XLeRobot GitHub集成使用指南

## 🎯 概述

本文档介绍XLeRobot项目与GitHub仓库的安全集成配置，确保敏感信息得到妥善保护的同时，实现代码的版本控制和协作开发。

## 🔐 安全配置概览

### 环境文件结构
```
.env                    # 安全模板文件（可提交，不含真实密钥）
.env.example           # 完整配置模板和使用说明
.env.local             # 本地开发配置（含真实密钥，不提交）
.env.development       # 开发环境配置
.env.testing           # 测试环境配置
.env.production        # 生产环境配置（服务器专用）
```

### Git安全规则
- ✅ 所有敏感环境文件已添加到`.gitignore`
- ✅ API密钥文件、证书文件自动保护
- ✅ 备份文件、临时文件防止意外提交
- ✅ 多层次安全保护机制

## 📋 快速开始

### 1. 环境配置管理

#### 创建开发环境配置
```bash
# 从模板创建开发环境配置
python3.10 scripts/manage_env.py create --env development

# 编辑开发环境配置
nano .env.development
```

#### 验证环境配置
```bash
# 验证配置文件格式和必需项
python3.10 scripts/manage_env.py validate --env development

# 列出所有环境文件状态
python3.10 scripts/manage_env.py list
```

#### 加载环境配置
```bash
# 加载指定环境配置
python3.10 scripts/manage_env.py load --env development

# 或使用传统方式
source .env.development
```

### 2. GitHub同步操作

#### 安全同步（推荐）
```bash
# 交互式同步（自动备份，选择策略）
python3.10 scripts/sync_with_github.py

# 或指定同步策略
python3.10 scripts/sync_with_github.py --strategy merge   # 安全合并
python3.10 scripts/sync_with_github.py --strategy reset   # 重置到远程
python3.10 scripts/sync_with_github.py --strategy diff    # 仅查看差异
```

#### 传统Git操作
```bash
# 查看状态
git status

# 添加非敏感文件
git add docs/ scripts/ src/ tests/

# 提交更改
git commit -m "your commit message"

# 推送到远程
git push origin main
```

## 🔧 详细使用指南

### 环境变量管理脚本

#### `manage_env.py` 功能说明

| 命令 | 说明 | 示例 |
|------|------|------|
| `create` | 从模板创建环境配置 | `python3.10 scripts/manage_env.py create --env testing` |
| `validate` | 验证配置文件完整性 | `python3.10 scripts/manage_env.py validate --env local` |
| `load` | 加载环境配置到当前会话 | `python3.10 scripts/manage_env.py load --env production` |
| `list` | 列出所有环境文件状态 | `python3.10 scripts/manage_env.py list` |
| `backup` | 备份所有环境配置文件 | `python3.10 scripts/manage_env.py backup` |

#### 环境配置最佳实践

1. **开发环境** (`.env.development`)
   - 使用开发API密钥
   - 启用调试模式
   - 宽松的安全设置

2. **测试环境** (`.env.testing`)
   - 使用测试API密钥
   - 模拟数据支持
   - 自动化测试友好的配置

3. **生产环境** (`.env.production`)
   - 使用生产API密钥
   - 严格的安全设置
   - 优化的性能配置

### GitHub同步脚本

#### `sync_with_github.py` 同步策略

| 策略 | 说明 | 使用场景 |
|------|------|----------|
| **merge** | 安全合并，保留本地更改 | 有本地重要修改需要保留 |
| **reset** | 重置到远程版本 | 本地修改不重要，要获取最新代码 |
| **diff** | 仅查看差异，不执行同步 | 想要先查看再决定 |

#### 同步安全特性

1. **自动备份**
   - 未提交修改自动保存到stash
   - 创建备份分支保护当前状态
   - 时间戳标记便于追踪

2. **安全检查**
   - 验证`.gitignore`配置
   - 检查敏感文件是否被意外添加
   - 确认环境文件保护状态

3. **差异分析**
   - 显示文件更改统计
   - 展示详细代码差异
   - 提供决策依据

## 🚀 开发工作流

### 日常开发流程

1. **开始开发前**
   ```bash
   # 同步最新代码
   python3.10 scripts/sync_with_github.py

   # 加载开发环境配置
   source .env.development
   ```

2. **开发过程中**
   ```bash
   # 定期保存进度
   git add .
   git commit -m "progress: update feature xyz"

   # 保护敏感信息
   python3.10 scripts/manage_env.py validate --env development
   ```

3. **完成开发后**
   ```bash
   # 验证所有配置
   python3.10 scripts/manage_env.py list

   # 同步到远程仓库
   git push origin main
   ```

### 团队协作流程

1. **新成员加入**
   ```bash
   # 克隆仓库
   git clone https://github.com/jodykwong/rdk-x5-xlerobot.git

   # 创建本地配置
   python3.10 scripts/manage_env.py create --env local

   # 填入个人API密钥
   nano .env.local
   ```

2. **代码审查**
   ```bash
   # 检查敏感信息
   python3.10 scripts/manage_env.py validate --env local

   # 确认Git状态
   git status

   # 查看提交差异
   git diff --stat HEAD~1
   ```

## 🔒 安全最佳实践

### 环境变量管理

1. **密钥轮换**
   ```bash
   # 备份当前配置
   python3.10 scripts/manage_env.py backup

   # 更新API密钥
   nano .env.local

   # 验证新配置
   python3.10 scripts/manage_env.py validate --env local
   ```

2. **多环境隔离**
   - 不同环境使用不同的API密钥
   - 生产环境密钥仅限服务器访问
   - 定期轮换所有API密钥

3. **配置验证**
   ```bash
   # 开发前验证
   python3.10 scripts/manage_env.py validate --env development

   # 部署前验证
   python3.10 scripts/manage_env.py validate --env production
   ```

### Git安全操作

1. **提交前检查**
   ```bash
   # 检查敏感文件
   git status --porcelain | grep -E "\.env|key|secret"

   # 确认.gitignore生效
   git check-ignore .env.local
   ```

2. **定期安全审计**
   ```bash
   # 检查仓库历史
   git log --oneline --grep="password\|secret\|key"

   # 验证当前配置
   python3.10 scripts/sync_with_github.py --strategy diff
   ```

## 🆘 故障排查

### 常见问题

#### 1. 环境文件无法加载
```bash
# 检查文件权限
ls -la .env.local

# 验证文件格式
python3.10 scripts/manage_env.py validate --env local

# 重新创建配置
python3.10 scripts/manage_env.py create --env local
```

#### 2. Git同步冲突
```bash
# 查看冲突文件
git status

# 安全解决冲突
python3.10 scripts/sync_with_github.py --strategy merge

# 或重置到远程版本
python3.10 scripts/sync_with_github.py --strategy reset
```

#### 3. 敏感信息意外提交
```bash
# 立即回滚最近的提交
git reset --hard HEAD~1

# 修改.gitignore确保安全
echo ".env.local" >> .gitignore

# 强制推送到远程（谨慎操作）
git push --force-with-lease origin main
```

### 获取帮助

1. **查看脚本帮助**
   ```bash
   python3.10 scripts/manage_env.py --help
   python3.10 scripts/sync_with_github.py --help
   ```

2. **检查项目状态**
   ```bash
   # 完整环境检查
   ./start_voice_assistant.sh check

   # 验证Git配置
   git remote -v
   git status
   ```

3. **联系支持**
   - 查看项目文档: `docs/`
   - 运行诊断: `./start_voice_assistant.sh logs`
   - 提交Issue: GitHub仓库Issues页面

## 📚 相关文档

- [项目架构分析](architecture-analysis.md)
- [技术栈文档](tech-stack-documentation.md)
- [API集成指南](api-integration-guide.md)
- [部署配置说明](deployment-guide.md)

---

**最后更新**: 2025-11-23
**维护者**: BMad Master
**版本**: 1.0.0