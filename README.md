# Altimeter
## Описание проекта
Таксационный высотомер

Основные функции:
- Определение высоты объекта
- Вывод данных на экран устройства
- Запись на SD-карту 

## Приложение к проекту:
- Структурная и принципиальная схемы подключения компонентов устройства

## Схемы
![SCHEME]()
![SCHEME]()

## Принцип работы устройства:
Алгоритм работы устройства заключается в следующем. 
При включении устройства все показатели гироскопа-акселерометра калибруются и выставляются в ноль. С помощью кнопок на модуле с ЖК-дисплеем задается расстояние до измеряемого дерева. 
После этого с выбранного места производятся замеры угла от основания дерева до его макушки с помощью наведения прибора на соответствующие позиции.
Далее на основе геометрических законов разработанная программа вычисляет высоту ствола дерева и выводит показания на экран. 
Все произведенные измерения записываются на SD-карту. В файле, в который записываются показания, указывается номер измеряемого дерева и его высота. 
Данные на SD-карте защищены от перезаписи тем, что при включении питания устройства создается новый текстовый файл с названием.

## Шаги по прошивке:
* Произвести коммутацию используемых модулей устройства по схеме
* Подключить Arduino к компьютеру
* Запустить файл прошивки (расширение .ino)
* Настроить IDE (COM порт, модель Arduino)
* Загрузить прошивку в устройство

----
Устройство разработано в рамках выпускной квалификационной работы на тему "Автоматизированное устройство измерения высоты объекта" в Костромском Государственном Университете.