from paho.mqtt import client as mqtt_client

class MqttClient:
    def __init__(self):
        self.clientId = "kinect_pose"
        self.port = 1883
        self.broker = "localhost"
        self.connect()

    def connect(self):
        client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, self.clientId)
        client.connect(self.broker, self.port, 60)
        self._client = client


    def pub(self, topic: str, payload: str):
        self.client.publish(topic, payload)

    @property
    def client(self):
        return self._client