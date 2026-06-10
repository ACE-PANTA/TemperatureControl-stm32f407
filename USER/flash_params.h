#ifndef __FLASH_PARAMS_H
#define __FLASH_PARAMS_H

#include "sys.h"
#include "app_config.h"

/* ============================================================
 * Flash 参数持久化
 *
 *   Flash 扇区: Sector_7 (0x08060000, 128KB, 最后一片)
 *   存储格式:   [Magic(4)] [Version(4)] [AppConfig数据] [Checksum(4)]
 *   保存策略:   标记脏后延迟 500ms 批量写入
 * ============================================================ */

/* 从 Flash 加载到 g_config (有效数据覆盖, 无效则保持默认) */
void Flash_Param_Load_Runtime(void);

/* 将 g_config 保存到 Flash, 返回 1=成功 */
u8   Flash_Param_Save(const AppConfig *cfg);

/* 标记需要保存 (延迟 500ms 后自动写入) */
void Flash_Param_MarkDirty(void);

/* 主循环调用, 处理延迟写入 */
void Flash_Param_Process(void);

#endif
