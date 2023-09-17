from ultralytics import YOLO

model = YOLO(r"best.pt")

model.predict(source ="trial6.jpg", show=True ,save=True)