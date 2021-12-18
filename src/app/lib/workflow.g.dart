// GENERATED CODE - DO NOT MODIFY BY HAND

part of 'workflow.dart';

// **************************************************************************
// JsonSerializableGenerator
// **************************************************************************

Workflow _$WorkflowFromJson(Map<String, dynamic> json) => Workflow(
      name: json['name'] as String,
      color: Workflow._stringToColor(json['color'] as String),
    );

Map<String, dynamic> _$WorkflowToJson(Workflow instance) => <String, dynamic>{
      'name': instance.name,
      'color': Workflow._colorToString(instance.color),
    };
