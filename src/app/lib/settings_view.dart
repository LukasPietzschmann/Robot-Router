import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:settings_ui/settings_ui.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'ws_wrapper/rosbridge.dart';

class LogsView extends StatefulWidget {
  LogsView({Key? key}) : super(key: key) {
    rb.connect('87.183.50.244', 9090);
  }

  final Rosbridge rb = Rosbridge();

  @override
  _LogsViewState createState() => _LogsViewState();
}

class SettingsView extends StatefulWidget {
  const SettingsView({Key? key}) : super(key: key);
  static const String IP_KEY = 'IP';

  @override
  _SettingsViewState createState() => _SettingsViewState();
}

class _SettingsViewState extends State<SettingsView> {
  String _ip = '';

  @override
  void initState() {
    super.initState();
    _loadIp();
  }

  void _loadIp() async {
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    setState(() {
      _ip = (prefs.getString(SettingsView.IP_KEY) ?? 'Not set');
    });
  }

  void _setIp(String ip) async {
    final SharedPreferences prefs = await SharedPreferences.getInstance();
    setState(() {
      _ip = ip;
      prefs.setString(SettingsView.IP_KEY, _ip);
    });
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
                                actions: <TextField>[
                                  TextField(
                                      keyboardType:
                                          const TextInputType.numberWithOptions(
                                              decimal: true),
                                      decoration: InputDecoration(
                                          border: const OutlineInputBorder(),
                                          labelText: 'IP Addresse',
                                          hintText: _ip),
                                      onSubmitted: (String newString) {
                                        _setIp(newString);
                                        Navigator.pop(context);
                                      }),
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

class _LogsViewState extends State<LogsView> {
  List<String> _events = <String>[];

  @override
  void initState() {
    Topic t = Topic(widget.rb, '/rosout', '/rosgraph_msgs/Log');
    t.subscribe((dynamic data) => {
          setState(() {
            dynamic convertedData = JsonDecoder().convert(data.toString());
            String newEvent =
                '${convertedData["op"]} on ${convertedData["topic"]}: ${convertedData["msg"]["msg"]}';
            _events = <String>[..._events, newEvent];
          })
        });
    super.initState();
  }

  @override
  void dispose() {
    widget.rb.closeConnection();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(title: const Text('Logs')),
        body: ListView(children: _events.map((String e) => Text(e)).toList()));
  }
}
