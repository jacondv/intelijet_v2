class UIStatus:
    def __init__(self,
                 name: str = "",
                 state_text: str = "",
                 detail: str = "",
                 style: dict = None):
        self.name = name
        self.state_text = state_text
        self.detail = detail
        self.style = style or {}  # dict chứa tất cả style, có thể áp dụng cho bất kỳ widget nào

