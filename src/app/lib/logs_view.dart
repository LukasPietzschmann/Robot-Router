import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:robot_router/settings_view.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'ws_wrapper/rosbridge.dart';

class LogsView extends StatefulWidget {
  LogsView({Key? key}) : super(key: key) {
    SharedPreferences.getInstance().then((SharedPreferences sp) {
      rb.connect(
        sp.getString(SettingsView.IP_KEY)!,
        9090,
      );
    });
  }

  final Rosbridge rb = Rosbridge();

  @override
  _LogsViewState createState() => _LogsViewState();
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
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(title: const Text('Logs')),
        body: ListView(children: _events.map((String e) => Text(e)).toList()));
  }
}
