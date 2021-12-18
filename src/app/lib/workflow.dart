import 'dart:ui';

import 'package:json_annotation/json_annotation.dart';

import 'extansions.dart';

part 'workflow.g.dart';

@JsonSerializable()
class Workflow {
  Workflow({required this.name, required this.color});

  factory Workflow.fromJson(Map<String, dynamic> json) =>
      _$WorkflowFromJson(json);

  Map<String, dynamic> toJson() => _$WorkflowToJson(this);

  static String _colorToString(Color c) => c.toHex();
  static Color _stringToColor(String c) => HexColor.fromHex(c);

  final String name;
  @JsonKey(toJson: _colorToString, fromJson: _stringToColor)
  final Color color;
}
