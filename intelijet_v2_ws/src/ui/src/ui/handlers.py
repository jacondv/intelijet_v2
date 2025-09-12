
from shared.config_loader import save_config, reload_config 
from ui.update_data_utils import load_ui_to_config, load_config_to_ui


def btnUpdateHousingParam_handler(app):

    # xử lý khi người dùng click UpdateHousingParam
    new_config = load_ui_to_config(app.ui.tab_setting)
    save_config(new_config)  # Lưu cấu hình vào file
    reload_config()
    load_config_to_ui(app.ui.tab_setting)  # Cập nhật lại UI để đảm bảo dữ liệu đúng