#include "TxMultiplexer.h"
#include "../util/log.h"

// TxMultiplexer 實現

TxMultiplexer::TxMultiplexer(ITransport* transport) 
    : transport_(transport), monitored_transport_(nullptr) {
    current_owner_[0] = '\0';  // 初始化為空字串
    
    // 創建監控傳輸裝飾器
    if (transport_) {
        monitored_transport_ = new MonitoredTransport(transport_, "TxMultiplexer");
        
        // 🗑️ 已移除協議統計回調 - 防止記憶體累積
    }
}

TxMultiplexer::~TxMultiplexer() {
    if (monitored_transport_) {
        delete monitored_transport_;
        monitored_transport_ = nullptr;
    }
}

ITransport* TxMultiplexer::acquire(const char* owner_name) {
    // 防呆：nullptr 轉為預設名稱
    if (!owner_name) owner_name = "UNKNOWN";
    
    if (current_owner_[0] != '\0') {
        LOGW("傳輸被 %s 占用，%s 請求被拒絕", current_owner_, owner_name);
        return nullptr;
    }
    
    if (!transport_ || !transport_->isReady()) {
        LOGE("傳輸未準備就緒，%s 請求失敗", owner_name);
        return nullptr;
    }
    
    // 安全複製到固定緩衝
    strncpy(current_owner_, owner_name, sizeof(current_owner_) - 1);
    current_owner_[sizeof(current_owner_) - 1] = '\0';
    
    LOGI("%s 獲得傳輸控制權", current_owner_);
    return monitored_transport_;  // 返回監控的傳輸
}

bool TxMultiplexer::release(const char* owner_name) {
    LOGI("🔍 release() 詳細診斷:");
    LOGI("   - 請求釋放者: %s", owner_name ? owner_name : "NULL");
    LOGI("   - 當前持有者: %s", current_owner_[0] != '\0' ? current_owner_ : "EMPTY");
    
    if (current_owner_[0] == '\0') {
        LOGW("❌ 傳輸無人持有，%s 釋放請求被忽略", owner_name ? owner_name : "NULL");
        LOGI("   - 分析: current_owner_ 為空，可能初始化時就沒有持有者");
        return false;
    }
    
    // 檢查是否為當前持有者（字串比較）
    if (owner_name == nullptr) {
        LOGE("❌ 釋放參數錯誤：owner_name 為 NULL，無法與 %s 比較", current_owner_);
        return false;
    }
    
    int strcmp_result = strcmp(current_owner_, owner_name);
    LOGI("   - 字串比較結果: strcmp('%s', '%s') = %d", current_owner_, owner_name, strcmp_result);
    
    if (strcmp_result != 0) {
        LOGE("❌ 權限錯誤：%s 試圖釋放 %s 的傳輸", owner_name, current_owner_);
        LOGE("   - 分析: 字串不匹配，可能是協議名稱不一致");
        return false;
    }
    
    // 釋放前刷新緩衝區
    LOGI("🧹 釋放前清理...");
    if (transport_ && transport_->isReady()) {
        transport_->flush();
        LOGI("✅ 傳輸緩衝區已刷新");
    } else {
        LOGW("⚠️ 無法刷新緩衝區 (transport=%s)", transport_ ? "NotReady" : "NULL");
    }
    
    current_owner_[0] = '\0';  // 清空緩衝
    LOGI("✅ %s 成功釋放傳輸控制權", owner_name);
    return true;
}

void TxMultiplexer::forceRelease() {
    if (current_owner_[0] != '\0') {
        LOGW("強制釋放 %s 的傳輸控制權", current_owner_);
        
        if (transport_ && transport_->isReady()) {
            transport_->flush();
            transport_->resetBuffers();
        }
        
        current_owner_[0] = '\0';  // 清空緩衝
    }
}

ITransport* TxMultiplexer::switchOwner(const char* old_owner, const char* new_owner) {
    // TxMultiplexer 切換詳細診斷
    LOGI("🔄 switchOwner 開始: %s → %s", old_owner ? old_owner : "NULL", new_owner ? new_owner : "NULL");
    LOGI("   - 當前持有者: %s", current_owner_[0] != '\0' ? current_owner_ : "EMPTY");
    LOGI("   - transport_ 狀態: %s", transport_ ? (transport_->isReady() ? "Ready" : "NotReady") : "NULL");
    
    // 特殊處理：初始狀態 (current_owner_ 為空 且 old_owner 為 "NONE")
    bool is_initial_state = (current_owner_[0] == '\0') && 
                           (old_owner && strcmp(old_owner, "NONE") == 0);
    
    if (is_initial_state) {
        LOGI("🎯 檢測到初始狀態切換: NULL → %s", new_owner ? new_owner : "NULL");
        LOGI("   - 跳過釋放步驟 (無人持有)");
        // 直接跳到獲取新控制權
    } else {
        // 正常釋放當前持有者
        LOGI("📤 嘗試釋放: %s", old_owner ? old_owner : "NULL");
        if (!release(old_owner)) {
            LOGE("❌ 協議切換失敗：無法釋放 %s", old_owner ? old_owner : "NULL");
            LOGE("   - current_owner_ = %s", current_owner_[0] != '\0' ? current_owner_ : "EMPTY");
            LOGE("   - 釋放失敗原因: 檢查 release() 方法內部邏輯");
            return nullptr;
        }
        LOGI("✅ 釋放成功: %s", old_owner ? old_owner : "NULL");
    }
    
    // 🚨 協議切換死機修復：暫停緩衝區清理
    // 清空緩衝區可能導致阻塞或死鎖
    LOGI("⚠️ 跳過緩衝區清理 (死機修復)");
    
    // 獲取新控制權
    LOGI("📨 嘗試獲取: %s", new_owner ? new_owner : "NULL");
    ITransport* result = acquire(new_owner);
    if (result) {
        LOGI("✅ switchOwner 完全成功: %s → %s", old_owner ? old_owner : "NULL", new_owner ? new_owner : "NULL");
    } else {
        LOGE("❌ switchOwner 獲取失敗: %s → %s", old_owner ? old_owner : "NULL", new_owner ? new_owner : "NULL");
    }
    return result;
}

size_t TxMultiplexer::writeWithStats(const uint8_t* data, size_t length, const char* protocol_name) {
    if (!monitored_transport_) return 0;
    
    size_t bytes_written = monitored_transport_->write(data, length);
    
    // 🗑️ 已移除協議統計記錄 - 防止記憶體累積
    
    return bytes_written;
}

void TxMultiplexer::recordProtocolActivity(const char* protocol_name, size_t bytes) const {
    // 🚨 死機測試：暫停統計功能，防止 std::map 和 std::string 累積
    // 20分鐘的高頻統計可能導致記憶體問題
    /*
    if (!protocol_name) return;
    
    std::string key(protocol_name);
    auto& stats = protocol_stats_[key];
    stats.recordPacket(bytes);
    */
}