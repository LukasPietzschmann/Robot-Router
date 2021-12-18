import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:robot_router/custom_blocks/block_with_subblock.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';

enum BlockType { t_action, t_passive, t_event, t_control }

Future<Block?> selectBlockFromList(BuildContext context) {
  return showModalBottomSheet<Block>(
    context: context,
    isDismissible: true,
    enableDrag: false,
    shape: ContinuousRectangleBorder(
      borderRadius: BorderRadius.circular(20),
    ),
    builder: (BuildContext context) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Expanded(
                child: ListView(
                    padding: const EdgeInsets.all(10),
                    children: Block.allBlocks().map((List<Block> blocks) {
                      return Wrap(direction: Axis.vertical, children: <Widget>[
                        Text(blocks[0].typeName),
                        ...blocks.map((Block block) => TextButton(
                            onPressed: () => Navigator.pop(context, block),
                            child: Text('    ' + block.name)))
                      ]);
                    }).toList()))
          ],
        ),
      );
    },
  );
}

abstract class InputProvider<T> extends StatelessWidget {
  const InputProvider({required this.onSubmit, Key? key}) : super(key: key);

  final void Function(T result) onSubmit;
}

class TextProvider extends InputProvider<String> {
  const TextProvider(
      {required void Function(String) onSubmit, this.hintText, Key? key})
      : super(onSubmit: onSubmit, key: key);

  final String? hintText;

  @override
  Widget build(BuildContext context) {
    return TextField(
        maxLines: 1,
        decoration: InputDecoration(
          border: InputBorder.none,
          hintText: hintText ?? 'Here goes your Text',
        ),
        onSubmitted: (String text) => onSubmit(text));
  }
}

class NumberProvider extends InputProvider<int> {
  const NumberProvider({required void Function(int) onSubmit, Key? key})
      : super(onSubmit: onSubmit, key: key);

  @override
  Widget build(BuildContext context) {
    return TextField(
        keyboardType: TextInputType.number,
        inputFormatters: <TextInputFormatter>[
          FilteringTextInputFormatter.digitsOnly
        ],
        maxLines: 1,
        decoration: const InputDecoration(
          border: InputBorder.none,
        ),
        onSubmitted: (String text) => onSubmit(int.parse(text)));
  }
}

class CappedValueProvider extends InputProvider<String> {
  const CappedValueProvider(
      {required this.possibleValues,
      required void Function(String) onSubmit,
      Key? key})
      : super(onSubmit: onSubmit, key: key);

  final List<String> possibleValues;

  @override
  Widget build(BuildContext context) {
    return PopupMenuButton<String>(
      tooltip: 'Select one of ${possibleValues.length} Values',
      onSelected: (String result) => onSubmit(result),
      itemBuilder: (BuildContext context) => <PopupMenuEntry<String>>[
        for (String value in possibleValues)
          PopupMenuItem<String>(child: Text(value), value: value)
      ],
    );
  }
}

abstract class Block extends StatefulWidget {
  const Block({Key? key}) : super(key: key);

  String get name;
  BlockType get type;
  String get typeName {
    return <String>[
      'Action Block',
      'Passive Block',
      'Event Block',
      'Control Block'
    ][type.index];
  }

  static List<List<Block>> allBlocks() {
    final List<Block> blocks = <Block>[
      const TestBlock(),
      const CommentBlock(),
      const BlockWithSubblock(),
    ];
    List<List<Block>> result = List<List<Block>>.generate(
        BlockType.values.length, (int index) => <Block>[]);
    for (Block block in blocks) {
      result[block.type.index].add(block);
    }
    result.removeWhere((List<Block> blocks) => blocks.isEmpty);
    return result;
  }

  @override
  BlockState createState();
}

abstract class BlockState extends State<Block> {
  bool _isCurrentlyRunning = false;

  bool get isCurrentlyRunning {
    return _isCurrentlyRunning;
  }

  set isCurrentlyRunning(bool newVal) {
    setState(() {
      _isCurrentlyRunning = newVal;
    });
  }

  Widget render();

  @override
  Widget build(BuildContext context) {
    return Card(
      margin: const EdgeInsets.all(10),
      shape: ContinuousRectangleBorder(
        side: BorderSide(
            color: _isCurrentlyRunning ? Colors.green : Colors.black, width: 1),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Padding(padding: const EdgeInsets.all(15), child: render()),
    );
  }
}
