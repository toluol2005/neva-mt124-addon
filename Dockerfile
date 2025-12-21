ARG BUILD_FROM=ghcr.io/home-assistant/amd64-base-python:3.12-alpine-3.20  # Подстройте под вашу архитектуру
FROM ${BUILD_FROM}

# Установка зависимостей
RUN pip install --no-cache-dir pyserial paho-mqtt

# Копируем скрипт
COPY run.py /

CMD ["python3", "/run.py"]
