# DepthMapBeta
Работающий вариант построения карты глубины: videoCameraTest()

### Основные проблеммы:
  1. Значения которые возвращаются не пркирепленны ни к одной системе исчисления
  2. Не правильноопределяется глубина, дже в рамках непрвильных значений
  
  Пытался найти ошибкув архитектуре, но в самом учебнике и в интернете используют похожие архитектуры. Так что скорее всего
ошибка в мелкойнастройке.

  Впревую очередь, нужно улучшить качество получаемой карты диспаратности, так как полоучаемые значения очень не стабильны и
и очень зависятот освещенности. Так просто пограв со светом на ладони значения получались от 4 до 180 условных единиц, 
что говорит о нестабильности получаемых значений.

  Я увидел, что можно попробовать улучшить карту диспаратности, просто пройтись сверточным фильтром(Детектором граней) наложить
наложить полученное изображение на исходное изображение. И получать карту диспаратности от получившегося. Также хочу поробовать
различные фильтры. В плане самойархитектуры пока мыслей нет.
