import 'dart:io';
import 'dart:math' as math;
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';
import 'workflow.dart';
import 'extansions.dart';

Future<String> get _appPath async {
  final Directory directory = await getApplicationDocumentsDirectory();

  return directory.path;
}

Future<File> get _workflow_file async {
  final String path = await _appPath;
  return File('$path/workflows.json');
}

List<Workflow> get workflows {
  return <Workflow>[
    Workflow.fromJson({'name': 'WF1', 'color': Colors.blue.toHex()}),
    Workflow.fromJson({'name': 'WF2', 'color': Colors.yellow.toHex()}),
    Workflow.fromJson({'name': 'WF3', 'color': Colors.red.toHex()})
  ];
}
