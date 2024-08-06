from paho.mqtt import client as mqtt_client

class MqttClient:
    def __init__(self):
        self.clientId = "kinect_ros_client"
        self.port = 1883
        self.broker = "192.168.0.10"
        self.connect()

    def connect(self):
        client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, self.clientId)
        client.username_pw_set("kinect_ros_client", "kinect_ros_password")
        client.connect(self.broker, self.port, 60)
        self._client = client

    def pub(self, topic: str, payload: str):
        self.client.publish(topic, payload)

    @property
    def client(self):
        return self._client