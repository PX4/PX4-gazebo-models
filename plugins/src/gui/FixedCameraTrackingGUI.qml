import QtQuick 2.15
import QtQuick.Controls 2.15
import GzGui 1.0

Item {
    width: 200
    height: 100

    Column {
        spacing: 10

        Text { text: "Select Target:" }

        ComboBox {
            id: targetSelector
            model: ["drone_1", "drone_2", "ground_target"]
            onCurrentIndexChanged: FixedCameraTracking.SetTarget(currentText)
        }
    }
}
