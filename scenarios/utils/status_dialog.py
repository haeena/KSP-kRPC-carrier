import krpc

class StatusDialog(object):

    #TODO: make this singleton?
    def __init__(self, conn: krpc.connection = None):
        if not conn:
            conn = krpc.connect(name='Status Dialog')

        # setup canvas
        canvas = conn.ui.stock_canvas
        screen_size = canvas.rect_transform.size
        self.panel = canvas.add_panel()
        panel = self.panel

        rect = panel.rect_transform
        rect.size = (400, 50)
        rect.position = (250-(screen_size[0]/2), 400)

        self.text_status = panel.add_text("")
        text_status = self.text_status
        text_status.rect_transform.size = (380,40)
        text_status.rect_transform.position = (0, 0)
        text_status.color = (1, 1, 1)
        text_status.size = 12

    def status_update(self, message):
        status_line = "Status: {}".format(message)
        print(status_line)
        self.text_status.content = status_line

if __name__ == "__main__":
    import time
    sd = StatusDialog()
    sd.status_update("this is test, how log messages can we go? I don't know but let's do some test")
    time.sleep(5)
