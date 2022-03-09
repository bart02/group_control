Уход от столкновения (тестовое задание)
==================================
### Описание дисциплины

[Общая информация](INFO.md)

В рамках дисциплины «Уход от столкновения» задача Команды разработать и запрограммировать алгоритм управления, позволяющий беспилотному летательному аппарату пролететь заданный маршрут, избежав столкновений с динамическими летящими препятствиями (БЛА/птицами).  В тестовом задании группа беспилотных аппаратов состоит из одинаковых аппаратов в количестве определяемом заданием. Аппараты являются цифровыми двойниками реальных моделей беспилотных летательных аппаратов мультироторного типа.

Полигон представляет собой участок горной местности.

Маршрут представляет собой ломаную линию, заданную координатами ее вершин (точек маршрута). Полет по маршруту начинается при пересечении аппаратом стартовой плоскости и заканчивается при пересечении аппаратом финишной плоскости. При полете по маршруту требуется строгое поддержание траектории. Допустимая зона находится вокруг линии маршрута не дальше некоторого расстояния R от нее. Значение R и координаты точек маршрута выдаются автоматически при запуске Симулятора.

В качестве динамических препятствий выступают БЛА/птицы, движущиеся по своим траекториям, не зависящим от перемещений аппарата.

### Запуск

Команда для запуска Симулятора выглядит следующим образом

```
./avoid.sh РЕЖИМ_ЛИГИ КОЛ_ВО
```

где РЕЖИМ_ЛИГИ может быть prof для Профессиональной или exp для Экспертной лиги,
КОЛ_ВО - общее количество аппаратов (по умолчанию 1).

Пример запуска в режиме Профессиональной лиги

```
./avoid.sh prof
```

В тестовом задании группа беспилотных аппаратов состоит из одинаковых аппаратов в количестве 1 штуки.

### Задание

Задача Команды разработать алгоритм управления аппаратом, который автоматически:

* выдаст управляющее воздействие на аппарат для перемещения по маршруту;
* определит динамические препятствия, с которыми может произойти столкновение;
* выдаст управляющее воздействие для избегания столкновений.

Для Профессиональной Лиги положение динамических препятствий, а также их габаритные размеры выдаются в течение полета аппарата, при нахождении препятствия на расстоянии обнаружения L от аппарата. Значение L выдается автоматически при запуске Симулятора.

Для Экспертной Лиги параметры динамических препятствий необходимо определять с помощью оптических камер аппарата.

При определении победителя оценивается число столкновений с препятствиями и время полета. Побеждает команда, набравшая наибольшее число баллов.

Баллы начисляются или снимаются за следующие действия:

* общее время полета по маршруту;
* среднеквадратичное отклонение траектории аппарата от маршрута;
* число столкновений с препятствиями;
* время нахождения аппарата вне допустимой зоны (критический критерий, при высоких значениях параметра жюри может принять решение о признании результата недействительным).

#### Параметры тестового задания

Координаты точек маршрута (широта, долгота, высота): tasks/avoid/gps_mission_1.rt

Допустимое расстояние R вокруг линии маршрута (метры): 5

Габаритные размеры препятствий (метры): 1.0 х 1.0 х 0.3

Расстояние обнаружения препятствий (для Профессиональной лиги) L (метры): 50

Выдача препятствий в ROS топике (для Профессиональной лиги):

* `/ca_radar/rel_obstacles`

Формат строки публикуемой в топик, значения разделены пробелами:

```
ВРЕМЯ_СЕК НОМЕР_АППАРАТА1 e1 n1 u1 НОМЕР_АППАРАТА1 e2 n2 u2 НОМЕР_АППАРАТА2 e3 n3 u3 НОМЕР_АППАРАТА2 e4 n4 u4 ...
```

где ВРЕМЯ_СЕК - время получения данных, в секундах, (e,n,u) - относительный вектор от аппарата до препятствия в базисе Восток-Север-Верх.

### Определение отобранных команд

Для отбора в финал нужно зарегистрироваться до 20 ноября и прислать решение тестовой задачи до 22 ноября.

### Загрузка решения

Решения тестовых заданий необходимо отправлять через форму https://forms.gle/RsRcncF4uQpXgAtVA (требуется учетная запись пользователя Google).