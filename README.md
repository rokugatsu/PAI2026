rtx: NUCで実行するロボットの走行制御node群\
    1. localizer: Pointcloud MapとのマッチングによるLocalizationをおこなう.\
    2．waypoint_plannner: Localization結果をもとに，waypointと周囲のpointcloud，および走行可能領域から走行目標点を設定する.\
    3．motion_coordinator: waypoint_plannerが発行した走行目標点に追従するよう速度・角速度制御をおこなう．また，Waypointに設定された一時停止や信号認識横断などを実行する．

recognizer: Jetson Orin Nano でmodelを用いた推論を実行するnode．2種類の推論をYOLOv8のmodelを切り替えることで実行．\
    1．歩行者認識：　”COCOmodel”を使用.\
    2．信号認識：独自データセットによる信号認識modelを使用.
