import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';

class BlockWithSubblock extends Block {
  const BlockWithSubblock({Key? key}) : super(key: key);

  @override
  BlockState createState() => BlockWithSubblockState();

  @override
  String get name => 'Block with subblock';

  @override
  BlockType get type => BlockType.t_action;

  @override
  T accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitBlockWithSubblock(this);
  }
}

class BlockWithSubblockState extends BlockState {
  Block? _subblock;

  @override
  Widget render() {
    return Row(
      children: <Widget>[
        const Text('This is a Block:'),
        Expanded(
          child: InkWell(
              child: Card(
                  color: Colors.grey,
                  shape: ContinuousRectangleBorder(
                    side: const BorderSide(color: Colors.black45, width: 1),
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: Padding(
                    padding: const EdgeInsets.all(10),
                    child: _subblock ??
                        const Text('Click the grey area to select a block',
                            style: TextStyle(color: Colors.white)),
                  )),
              onTap: () {
                selectBlockFromList(context).then((Block? block) => {
                      setState(() {
                        _subblock = block ?? _subblock;
                      })
                    });
              }),
        ),
      ],
    );
  }
}
