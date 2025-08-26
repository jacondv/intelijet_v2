
from shared.config_loader import save_config, reload_config 
from ui.update_data_utils import upload_data_to_ui, download_data_from_ui


def btnUpdateHousingParam_handler(app):

    # xử lý khi người dùng click UpdateHousingParam
    new_config = download_data_from_ui(app.ui.tab_setting)
    save_config(new_config)  # Lưu cấu hình vào file
    reload_config()
    upload_data_to_ui(app.ui.tab_setting)  # Cập nhật lại UI để đảm bảo dữ liệu đúng