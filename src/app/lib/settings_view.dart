import 'package:flutter/material.dart';
import 'package:settings_ui/settings_ui.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'logs_view.dart';

class SettingsView extends StatefulWidget {
  const SettingsView({Key? key}) : super(key: key);

  @override
  _SettingsViewState createState() => _SettingsViewState();
}

class _SettingsViewState extends State<SettingsView> {
  String _ip = '';

  @override
  void initState() {
    SharedPreferences.setMockInitialValues({});
    internal() async {
      SharedPreferences sp = await SharedPreferences.getInstance();
      setState(() {
        _ip = sp.getString('IP') ?? 'Not Set';
      });
    }

    internal();

    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Setting')),
      body: Column(
        children: <Widget>[
          Expanded(
            child: SettingsList(
              platform: DevicePlatform.iOS,
              lightTheme: const SettingsThemeData(
                  settingsListBackground: Colors.white10),
              sections: [
                SettingsSection(
                  title: const Text('Common'),
                  tiles: <SettingsTile>[
                    SettingsTile.navigation(
                      title: const Text('IP Address'),
                      description:
                          const Text('Enter the IP address of the robot'),
                      value: Text(_ip),
                      leading: const Icon(Icons.password),
                      onPressed: (BuildContext context) {
                        showDialog(
                            context: context,
                            builder: (BuildContext context) {
                              return AlertDialog(
                                title: const Text('Enter IP'),
                                actions: <Widget>[
                                  TextField(
                                      decoration: const InputDecoration(
                                          border: OutlineInputBorder(),
                                          labelText: 'IP Addresse'),
                                      onChanged: (String newString) =>
                                          setState(() {
                                            _ip = newString;
                                          })),
                                  TextButton(
                                    onPressed: () {
                                      SharedPreferences.getInstance().then(
                                          (SharedPreferences sp) =>
                                              sp.setString('IP', _ip));
                                      Navigator.pop(context, false);
                                    },
                                    child: const Text('Confirm'),
                                  )
                                ],
                              );
                            });
                      },
                    )
                  ],
                ),
              ],
            ),
          ),
          ElevatedButton(
              onPressed: () {
                Navigator.push(
                    context,
                    MaterialPageRoute(
                        builder: (BuildContext context) => LogsView()));
              },
              child: const Text('I am a dev.'))
        ],
      ),
    );
  }
}
